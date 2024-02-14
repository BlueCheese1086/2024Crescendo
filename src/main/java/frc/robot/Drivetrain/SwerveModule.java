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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
    // Analog Encoder + offset
    private final DutyCycleEncoder cancoder;
    private final double offset;

    // Motors
    private final CANSparkMax turn;
    private final CANSparkMax drive;

    // Motor Encoders
    private final RelativeEncoder turnEnc;
    private final RelativeEncoder driveEnc;

    // Spark PID Controllers
    private final SparkPIDController turnPID;
    private final SparkPIDController drivePID;

    // Public vars
    public SwerveModuleState state;
    public SwerveModulePosition position;
    public String name;

    public SwerveModule(String name, int driveId, int turnId, int channel, double offset) {
        // Saving the name of the module.
        this.name = name;

        // Initializing the cancoder and saving its offset.
        cancoder = new DutyCycleEncoder(channel);
        this.offset = offset;

        // Initializing the motors.
        turn = new CANSparkMax(turnId, MotorType.kBrushless);
        drive = new CANSparkMax(driveId, MotorType.kBrushless);

        // Resetting the settings of the motors.
        turn.restoreFactoryDefaults();
        drive.restoreFactoryDefaults();

        // Setting an amp limit.
        turn.setSmartCurrentLimit(35);
        drive.setSmartCurrentLimit(35);

        // Enabling voltage compansation.
        drive.enableVoltageCompensation(12);
        turn.enableVoltageCompensation(12);

        // Inverting the motors as necessary.
        turn.setInverted(true);
        drive.setInverted(false);

        // Putting the motors into Coast mode.
        turn.setIdleMode(IdleMode.kCoast);
        drive.setIdleMode(IdleMode.kCoast);

        // Gettting the encoders of each motor.
        turnEnc = turn.getEncoder();
        driveEnc = drive.getEncoder();

        // Resetting the positions of each encoder.
        driveEnc.setPosition(0);
        turnEnc.setPosition(0);

        // Getting the PID Controllers of each motor.
        turnPID = turn.getPIDController();
        drivePID = drive.getPIDController();

        // Setting PIDFF values for the turn motor.
        turnPID.setP(DriveConstants.turnP);
        turnPID.setI(DriveConstants.turnI);
        turnPID.setD(DriveConstants.turnD);

        // Setting PIDFF calues for the drive motor.
        drivePID.setP(DriveConstants.driveP);
        drivePID.setI(DriveConstants.driveI);
        drivePID.setD(DriveConstants.driveD);
        drivePID.setFF(DriveConstants.driveFF);

        // Setting RPM conversions for each encoder.
        turnEnc.setPositionConversionFactor(360.0 / DriveConstants.turnRatio);
        driveEnc.setVelocityConversionFactor(DriveConstants.wheelCircumference / DriveConstants.driveRatio / 60);
        driveEnc.setPositionConversionFactor(DriveConstants.wheelCircumference / DriveConstants.driveRatio);

        // Saving the settings.
        turn.burnFlash();
        drive.burnFlash();

        this.state = new SwerveModuleState(0, getAngle());
        this.position = new SwerveModulePosition(0, getAngle());
    }

    /** Runs every 20 ms */
    @Override
    public void periodic() {
        // Updating the position of the Swerve Module
        this.position = new SwerveModulePosition(getDistance(), getAngle());
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
     * Gets the current module angle based on relative encoder
     * 
     * @return Rotation2d of the current module angle
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees((turnEnc.getPosition() % 360 + 360) % 360);
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
        this.state = SwerveModuleState.optimize(state, getAngle());

        drivePID.setReference(this.state.speedMetersPerSecond, ControlType.kVelocity);
        turnPID.setReference(getAdjustedAngle(this.state.angle.getDegrees()), ControlType.kPosition);
    }
}