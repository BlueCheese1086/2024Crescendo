package frc.robot.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
    // Cancoder
    private AnalogEncoder cancoder;
    private double offset;

    // Motors
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    // Encoder for each motor
    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    // PID Controllers for each motor
    private SparkPIDController drivePID;
    private SparkPIDController turnPID;

    public SwerveModule(String name, int driveID, int turnID, int cancoderID, double offset) {
        // Saving the offset of the cancoder
        this.offset = offset;

        // Setting the motors with the given IDs
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        // Setting the attributes of each motor
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(false);
        driveMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.burnFlash();

        turnMotor.restoreFactoryDefaults();
        driveMotor.setInverted(false);
        driveMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.burnFlash();

        // Getting the encoders for each motor
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        // Setting the attributes of each encoder
        driveEncoder.setPosition(0);
        driveEncoder.setVelocityConversionFactor(DriveConstants.wheelCircumference / DriveConstants.driveRatio / 60.0);
        driveEncoder.setPositionConversionFactor(DriveConstants.wheelCircumference / DriveConstants.driveRatio);
        turnEncoder.setPosition(getCancoderAngle().getDegrees());
        turnEncoder.setVelocityConversionFactor(360.0 / DriveConstants.turnRatio / 60.0);
        turnEncoder.setPositionConversionFactor(360.0 / DriveConstants.turnRatio);

        // Getting the PIDControllers for each motor
        drivePID = driveMotor.getPIDController();
        turnPID = turnMotor.getPIDController();

        // Setting PIDFF values for each PID
        drivePID.setP(DriveConstants.driveP);
        drivePID.setI(DriveConstants.driveI);
        drivePID.setD(DriveConstants.driveD);
        drivePID.setFF(DriveConstants.driveFF);

        turnPID.setP(DriveConstants.turnP);
        turnPID.setI(DriveConstants.turnI);
        turnPID.setD(DriveConstants.turnD);
        turnPID.setFF(DriveConstants.turnFF);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition()));
    }

    public Rotation2d getTurnAngle() {
        return new Rotation2d(turnEncoder.getPosition());
    }

    public Rotation2d getCancoderAngle() {
        // Calculating the angle of the cancoder and adjusting it if it is less than 0.
        double angle = (360 * (cancoder.getAbsolutePosition() - offset) % 360);
        angle += (angle < 0) ? 360 : 0;

        return new Rotation2d((360 * (cancoder.getAbsolutePosition() - this.offset) % 360));
    }

    // Speed in m/s, angle in radians
    public void setState(SwerveModuleState state) {
        // Optimizing the state so that the motor doesn't travel too far.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());

        // Setting the PID positions
        drivePID.setReference(optimizedState.speedMetersPerSecond * DriveConstants.maxSpeed, ControlType.kVelocity);
        turnPID.setReference(optimizedState.angle.getDegrees(), ControlType.kPosition);
    }
}