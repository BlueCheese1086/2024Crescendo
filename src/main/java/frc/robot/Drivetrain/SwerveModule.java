package frc.robot.Drivetrain;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class SwerveModule extends SubsystemBase {
    // Motors
    public TalonFX drive;
    private CANSparkMax turn;

    // Encoders
    private RelativeEncoder turnEncoder;
    private AnalogEncoder cancoder;

    // PID Controller
    private SparkPIDController turnPID;

    // Module Vars
    private SwerveModuleState state;
    private SwerveModulePosition position;
    private String name;

    public SwerveModule(String name, int driveID, int turnID, int cancoderID, double offset) {
        // Saving the name of the sparkmax
        this.name = name;

        // Initializing the motors
        drive = new TalonFX(driveID);
        turn = new CANSparkMax(turnID, MotorType.kBrushless);

        // Resetting motor configs
        turn.restoreFactoryDefaults();

        // Getting drive config
        TalonFXConfigurator driveConfigurator = drive.getConfigurator();
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        // Inverting the motors
        drive.setInverted(true);
        turn.setInverted(false);

        // Setting the neutral mode for the 
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turn.setIdleMode(IdleMode.kCoast);

        // Getting encoders for the motor
        turnEncoder = turn.getEncoder();

        // Getting PID Controllers for the motor
        turnPID = turn.getPIDController();

        // Setting PID values for each motor
        driveConfig.Slot0.kP = DriveConstants.driveP;
        driveConfig.Slot0.kI = DriveConstants.driveI;
        driveConfig.Slot0.kD = DriveConstants.driveD;
        driveConfig.Slot0.kS = DriveConstants.driveFF;

        turnPID.setP(DriveConstants.turnP);
        turnPID.setI(DriveConstants.turnI);
        turnPID.setD(DriveConstants.turnD);
        turnPID.setFF(DriveConstants.turnFF);

        // Initializing the cancoder
        cancoder = new AnalogEncoder(cancoderID);
        cancoder.setPositionOffset(offset);
        
        turnEncoder.setPosition(0);
        // turnEncoder.setPosition(cancoder.get());

        // Setting conversion values for the encoders
        turnEncoder.setVelocityConversionFactor(DriveConstants.turnVelConversionFactor);
        turnEncoder.setPositionConversionFactor(DriveConstants.turnPosConversionFactor);

        // Saving the configs for each motor
        driveConfigurator.apply(driveConfig);
        turn.burnFlash();
  
        this.state = new SwerveModuleState(getVelocity(), getAngle());
        this.position = new SwerveModulePosition(getDistance(), getAngle());
    }

    /** Runs every tick that the subsystem exists. */
    public void periodic() {
        // Uncomment to find cancoder offsets
        // setState(new SwerveModuleState(0, new Rotation2d()));
        this.state = new SwerveModuleState(getVelocity(), getAngle());
        this.position = new SwerveModulePosition(getDistance(), getAngle());
        SmartDashboard.putNumber(String.format("/%s/Analog_Encoder_Pos", name), cancoder.get());
        SmartDashboard.putNumber(String.format("/%s/offset", name), cancoder.getPositionOffset());

        double rotations = getAngle().getRotations();
        while (rotations < 0) rotations++;
        rotations -= (int) rotations;
        SmartDashboard.putNumber(String.format("/%s/Relative_Encoder_Pos", name), rotations);
    }

    /**
     * Gets the angle of the serve module.
     * 
     * @return The angle of the swerve module as a Rotation2d.
     */
    public Rotation2d getAngle() {
        // Getting radians
        double radians = turnEncoder.getPosition();

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
        double rotations = drive.getRotorPosition().getValueAsDouble();

        // Converting Rotations to Meters
        return rotations * DriveConstants.drivePosConversionFactor;
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
        double rps = drive.getRotorVelocity().getValueAsDouble();

        // Converting r/s to m/s
        return rps * DriveConstants.driveVelConversionFactor;
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
        drive.setControl(new VelocityDutyCycle(newState.speedMetersPerSecond / DriveConstants.drivePosConversionFactor));
        turnPID.setReference(getAdjustedAngle(newState.angle).getRadians(), ControlType.kPosition);
    }
}