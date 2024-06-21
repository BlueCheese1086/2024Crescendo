package frc.robot.Drivetrain;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.fasterxml.jackson.databind.deser.std.StdScalarDeserializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class SparkMaxSwerveModule extends SubsystemBase {
    // Motors
    private CANSparkMax drive;
    private CANSparkMax turn;

    // Encoders
    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;
    private AnalogEncoder cancoder;
    private double offset;

    // PID Controllers
    private SparkPIDController drivePID;
    private SparkPIDController turnPID;

    // Module Vars
    private SwerveModuleState state;
    private SwerveModulePosition position;
    private String name;

    public SparkMaxSwerveModule(String name, int driveID, int turnID, int cancoderID, double offset) {
        this.name = name;
        // Initializing the motors
        drive = new CANSparkMax(driveID, MotorType.kBrushless);
        turn = new CANSparkMax(turnID, MotorType.kBrushless);

        // Resetting motor configs
        drive.restoreFactoryDefaults();
        turn.restoreFactoryDefaults();

        // Inverting the motors
        drive.setInverted(true);
        turn.setInverted(false);

        // Setting the neutral mode for the motors
        drive.setIdleMode(IdleMode.kBrake);
        turn.setIdleMode(IdleMode.kBrake);

        // Getting encoders for the motors
        driveEncoder = drive.getEncoder();
        turnEncoder = turn.getEncoder();

        driveEncoder.setPosition(0);

        // Setting conversion values for the encoders
        driveEncoder.setPositionConversionFactor(DriveConstants.drivePosConversionFactor);
        driveEncoder.setVelocityConversionFactor(DriveConstants.driveVelConversionFactor);
        turnEncoder.setPositionConversionFactor(DriveConstants.turnPosConversionFactor);
        turnEncoder.setVelocityConversionFactor(DriveConstants.turnVelConversionFactor);

        // Getting PID Controllers for the motors
        drivePID = drive.getPIDController();
        turnPID = turn.getPIDController();

        // Setting PID values for each motor
        drivePID.setP(DriveConstants.driveP);
        drivePID.setI(DriveConstants.driveI);
        drivePID.setD(DriveConstants.driveD);
        drivePID.setFF(DriveConstants.driveFF);

        turnPID.setP(DriveConstants.turnP);
        turnPID.setI(DriveConstants.turnI);
        turnPID.setD(DriveConstants.turnD);
        turnPID.setFF(DriveConstants.turnFF);

        // Initializing the cancoder
        cancoder = new AnalogEncoder(cancoderID);
        this.offset = offset;
        
        // Saving the configs for each motor
        drive.burnFlash();
        turn.burnFlash();

        // Setting the initial position of the turn encoder
        initializeEncoder();
  
        this.state = new SwerveModuleState(getVelocity(), getAngle());
        this.position = new SwerveModulePosition(getDistance(), getAngle());
    }

    // For some reason, this doesn't always work when used in the constructor.
    /**
     * This function sets the position of the turn encoder to the position of the absolute encoder, allowing the turnEncoder
     * to be more accurate when starting out.
     */
    public void initializeEncoder() {
        turnEncoder.setPosition((cancoder.getAbsolutePosition() - offset) * 2 * Math.PI);
    }

    /** Runs every tick that the subsystem exists. */
    public void periodic() {
        this.state = new SwerveModuleState(getVelocity(), getAngle());
        this.position = new SwerveModulePosition(getDistance(), getAngle());
    }

    /**
     * Gets the angle of the serve module.
     * 
     * @return The angle of the swerve module as a Rotation2d.
     */
    public Rotation2d getAngle() {
        // Getting radians
        double radians = turnEncoder.getPosition();

        // Putting the radians into the range of 0 to 2pi
        radians %= 2 * Math.PI;

        // Converting radians to Rotation2d
        return new Rotation2d(radians);
    }

    /**
     * Gets the distance the swerve module has traveled.
     * 
     * @return The distance the swerve module has traveled as a double
     */
    public double getDistance() {
        // Getting rotations
        double rotations = driveEncoder.getPosition();

        // Converting Rotations to Meters
        return rotations * DriveConstants.driveRatio * DriveConstants.wheelCircumference;
    }

    /**
     * Gets the state of the swerve module.
     * 
     * @return the state of the swerve module.
     */
    public SwerveModuleState getState() {
        return this.state;
    }

    /**
     * Gets the position of the swerve module.
     * 
     * @return the position of the swerve module.
     */
    public SwerveModulePosition getPosition() {
        return this.position;
    }

    /**
     * Gets the velocity of the swerve module.
     * 
     * @return The velocity of the swerve module as a double.
     */
    public double getVelocity() {
        // Getting Rotations/Second
        double rps = driveEncoder.getVelocity();

        // Converting r/s to m/s
        return rps * DriveConstants.driveRatio * DriveConstants.wheelCircumference;
    }

    /**
     * Optimizes the angle to keep the motor from flipping.
     * 
     * @param targetAngle The angle to optimize.
     * @return The optimized angle as a Rotation2d.
     */
    public Rotation2d getAdjustedAngle(Rotation2d targetAngle) {
        Rotation2d theta = getAngle().minus(targetAngle);

        while (theta.getRadians() >= Math.PI) {
            theta.minus(new Rotation2d(2.0 * Math.PI));
        }
        
        while (theta.getRadians() <= -Math.PI) {
            theta.plus(new Rotation2d(2.0 * Math.PI));
        }

        return getAngle().minus(theta);
    }

    public void setState(SwerveModuleState state) {
        // Optimizing the SwerveModuleState
        SwerveModuleState newState = SwerveModuleState.optimize(state, getAngle());

        SmartDashboard.putNumber(String.format( "/%s/speedMPS", name), state.speedMetersPerSecond);
        SmartDashboard.putNumber(String.format("/%s/angle", name), state.angle.getDegrees());
        SmartDashboard.putNumber(String.format("/%s/newSpeedMPS", name), newState.speedMetersPerSecond);
        SmartDashboard.putNumber(String.format("/%s/newAngle", name), newState.angle.getDegrees());

        // Setting the speed and position of each motor
        drivePID.setReference(newState.speedMetersPerSecond, ControlType.kVelocity);
        turnPID.setReference(getAdjustedAngle(newState.angle).getRadians(), ControlType.kPosition);
    }
}