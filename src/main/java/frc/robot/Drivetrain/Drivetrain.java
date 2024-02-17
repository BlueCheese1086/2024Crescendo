package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
    // The motors for the drivetrain subsystem
    CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.FrontLeftID, MotorType.kBrushless);
    CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.BackLeftID, MotorType.kBrushless);
    CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.FrontRightID, MotorType.kBrushless);
    CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.BackRightID, MotorType.kBrushless);

    // Getting encoders for the primary (front) motors.
    RelativeEncoder leftEncoder = frontLeftMotor.getEncoder();
    RelativeEncoder rightEncoder = frontRightMotor.getEncoder();

    // Sensors
    AHRS gyro = new AHRS(Port.kMXP);

    // Kinematics
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.kModuleToModuleDistance);

    // Odometry
    private final Pose2d initPose = new Pose2d(2, 7, new Rotation2d());
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), initPose);
    private final Field2d field = new Field2d();

    /**
     * Constructor. This method is called when an instance of the class is created. This should generally be used to set up
     * instance variables and perform any configuration or necessary set up on hardware.
     */
    public Drivetrain() {
        // Restoring the default settings.
        frontLeftMotor.restoreFactoryDefaults();
        backLeftMotor.restoreFactoryDefaults();
        frontRightMotor.restoreFactoryDefaults();
        backRightMotor.restoreFactoryDefaults();

        // Setting a current limit.
        frontLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
        backLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
        frontRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
        backRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);

        // Making the back motors follow the front.
        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);

        // Inverting motors as necessary.
        frontLeftMotor.setInverted(true);
        backLeftMotor.setInverted(true);
        frontRightMotor.setInverted(false);
        backRightMotor.setInverted(false);

        // Setting the idle modes.
        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        backLeftMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.setIdleMode(IdleMode.kBrake);
        backRightMotor.setIdleMode(IdleMode.kBrake);

        // Saving the settings
        frontLeftMotor.burnFlash();
        backLeftMotor.burnFlash();
        frontRightMotor.burnFlash();
        backRightMotor.burnFlash();
    }

    /**
     * This function runs every tick.
     * I am using it to update the odometry of the motors,
     * 
     */
    @Override
    public void periodic() {}

    /**
     * Drives the robot at the desired forward speed combined with the rotational speed.
     * 
     * @param xSpeed The desired forward speed of the robot.
     * @param zRotation The desired rotational speed of the robot.
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        // Applies a deadband to the inputs.
        MathUtil.applyDeadband(xSpeed, DriveConstants.deadband);
        MathUtil.applyDeadband(zRotation, DriveConstants.deadband);

        // Squares the inputs (while preserving the sign) to increase fine control while permitting full power.
        xSpeed = Math.abs(xSpeed) * xSpeed;
        zRotation = Math.abs(zRotation) * zRotation;

        // Creates the saturated speeds of the motors
        double leftSpeed = xSpeed - zRotation;
        double rightSpeed = xSpeed + zRotation;

        // Finds the maximum possible value of throttle + turn along the vector that the joystick is pointing, and then desaturates the wheel speeds.
        double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
        double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotation));
        if (greaterInput == 0.0) {
            leftSpeed = 0;
            rightSpeed = 0;
        } else {
            double saturatedInput = (greaterInput + lesserInput) / greaterInput;
            leftSpeed /= saturatedInput;
            rightSpeed /= saturatedInput;
        }

        // Sets the speed of the motors.
        frontLeftMotor.set(leftSpeed * DriveConstants.maxSpeed);
        frontRightMotor.set(rightSpeed * DriveConstants.maxSpeed);
    }

    /**
     * Returns the current angle of the robot
     * 
     * @return The angle of the robot as a Rotation2D
     */
    public Rotation2d getAngle() {
        return new Rotation2d();
    }
}