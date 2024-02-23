package frc.robot.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
    // Analog Encoder + offset
    private final DutyCycleEncoder cancoder;
    private final double offset;

    // Motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    // Motor Encoders
    private final RelativeEncoder driveEnc;
    private final RelativeEncoder turnEnc;

    // Spark PID Controllers
    private final SparkPIDController drivePID;
    private final SparkPIDController turnPID;

    // Public vars
    public String name;

    public SwerveModule(String name, int driveId, int turnId, int channel, double offset) {
        // Setting the name of the Swerve Module
        this.name = name;

        // Initializing the cancoder and saving its offset.
        cancoder = new DutyCycleEncoder(new DigitalInput(channel));
        this.offset = offset;

        // Initializing the motors.
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);

        // Resetting the settings of the motors.
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        // Inverting the motors as necessary.
        driveMotor.setInverted(false);
        turnMotor.setInverted(false);

        // Putting the motors into Coast mode.
        driveMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.setIdleMode(IdleMode.kCoast);

        // Gettting the encoders of each motor.
        driveEnc = driveMotor.getEncoder();
        turnEnc = turnMotor.getEncoder();

        // Resetting the positions of each encoder.
        driveEnc.setPosition(0);
        turnEnc.setPosition(0);

        // Setting RPM conversions for each encoder.
        driveEnc.setVelocityConversionFactor(DriveConstants.wheelCircumference / DriveConstants.driveRatio / 60);
        driveEnc.setPositionConversionFactor(DriveConstants.wheelCircumference / DriveConstants.driveRatio);
        turnEnc.setVelocityConversionFactor(360.0 / DriveConstants.turnRatio / 60);
        turnEnc.setPositionConversionFactor(360.0 / DriveConstants.turnRatio);
        
        // Getting the PID Controllers of each motor.
        drivePID = driveMotor.getPIDController();
        turnPID = turnMotor.getPIDController();

        // Setting PIDFF calues for the drive motor.
        drivePID.setP(DriveConstants.driveP);
        drivePID.setI(DriveConstants.driveI);
        drivePID.setD(DriveConstants.driveD);
        drivePID.setFF(DriveConstants.driveFF);
        
        // Setting PIDFF values for the turn motor.
        turnPID.setP(DriveConstants.turnP);
        turnPID.setI(DriveConstants.turnI);
        turnPID.setD(DriveConstants.turnD);
        turnPID.setFF(DriveConstants.turnFF);

        // Saving the settings.
        driveMotor.burnFlash();
        turnMotor.burnFlash();
    }

    /** Runs every 20 ms */
    @Override
    public void periodic() {
        // Updating the position of the Swerve Module
        SmartDashboard.putNumber(String.format("/Drivetrain/%s/Distance", name), getDistance());
        SmartDashboard.putNumber(String.format("/Drivetrain/%s/Velocity", name), getVelocity());
        SmartDashboard.putNumber(String.format("/Drivetrain/%s/Angle", name), getAngle().getDegrees());
    }

    /** Initializes the turn encoders to match the cancoder. */
    public void initEncoder() {
        turnEnc.setPosition((cancoder.getAbsolutePosition() - offset) * 360.0);
    }

    /**
     * Gets the distance the module has traveled.
     * 
     * @return How far the module has traveled in meters.
     */
    public double getDistance() {
        return driveEnc.getPosition();
    }

    /**
     * Gets the velocity that the module is running at.
     * 
     * @return The velocity of the motor in meters/second.
     */
    public double getVelocity() {
        return driveEnc.getVelocity();
    }

    /**
     * Gets the current module angle based on relative encoder
     * 
     * @return Rotation2d of the current module angle
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(turnEnc.getPosition() % 360);
    }

    /**
     * Calculate the angle motor setpoint based on the desired angle and the current
     * angle measurement
     * 
     * @return Returns the calculated delta angle
     */
    public double getAdjustedAngle(double targetAngle) {
        double theta = getAngle().getDegrees() - targetAngle;

        if (theta > 180)  theta -= 360;
        if (theta < -180) theta += 360;

        return turnEnc.getPosition() - theta;
    }

    /**
     * Sets the module to the desired state
     * 
     * @param state The desired state
     */
    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getAngle());

        drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        System.out.println(name + ": " + state.angle.getDegrees());
        turnPID.setReference(getAdjustedAngle(state.angle.getDegrees()), ControlType.kPosition);
    }

    /**
     * Gets the current state of the swerve module.
     * 
     * @return The state of the swerve module (m/s, rot2d)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Gets the current position of the swerve module.
     * 
     * @return The position of the swerve module (m, rot2d)
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }
}