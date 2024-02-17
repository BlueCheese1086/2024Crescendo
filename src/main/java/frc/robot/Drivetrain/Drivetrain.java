package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    // The motors for the drivetrain subsystem
    private CANSparkMax frontLeftMotor;
    private CANSparkMax backLeftMotor;
    private CANSparkMax frontRightMotor;
    private CANSparkMax backRightMotor;

    // Getting encoders for the primary (front) motors.
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    // Gyro
    private AHRS gyro;

    // Kinematics
    public final DifferentialDriveKinematics kinematics;

    // Odometry
    private final Pose2d initPose;
    private final DifferentialDriveOdometry odometry;

    /**
     * Constructor. This method is called when an instance of the class is created. This should generally be used to set up
     * instance variables and perform any configuration or necessary set up on hardware.
     */
    public Drivetrain() {
        // Initializing the motors
        frontLeftMotor = new CANSparkMax(DriveConstants.FrontLeftID, MotorType.kBrushless);
        backLeftMotor = new CANSparkMax(DriveConstants.BackLeftID, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(DriveConstants.FrontRightID, MotorType.kBrushless);
        backRightMotor = new CANSparkMax(DriveConstants.BackRightID, MotorType.kBrushless);

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

        // Initializing the Encoders
        leftEncoder = frontLeftMotor.getEncoder();
        rightEncoder = frontRightMotor.getEncoder();

        // Setting left encoder conversion values
        leftEncoder.setPositionConversionFactor(Math.pow(Units.inchesToMeters(DriveConstants.wheelCircumference), 2) * Math.PI);
        leftEncoder.setVelocityConversionFactor(Math.pow(Units.inchesToMeters(DriveConstants.wheelCircumference), 2) * Math.PI / 60);

        // Setting right encoder conversion values
        rightEncoder.setPositionConversionFactor(Math.pow(Units.inchesToMeters(DriveConstants.wheelCircumference), 2) * Math.PI);
        rightEncoder.setVelocityConversionFactor(Math.pow(Units.inchesToMeters(DriveConstants.wheelCircumference), 2) * Math.PI / 60);
        
        // Initializing Gyro
        gyro = new AHRS(Port.kMXP);

        // Initializing Kinematics
        kinematics = new DifferentialDriveKinematics(DriveConstants.kModuleToModuleDistance);
        initPose = new Pose2d(2, 7, new Rotation2d());
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), initPose);

        // Implementing PathPlanner
        AutoBuilder.configureRamsete(
            this::getPose,
            this::resetPose,
            this::getSpeeds,
            this::drive,
            new ReplanningConfig(),
            () -> {
                if (DriverStation.getAlliance().isPresent()) {
                    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                }

                return false;
            },
            null
        );
    }

    /**
     * This function runs every tick.
     * I am using it to update the odometry of the motors,
     * 
     */
    @Override
    public void periodic() {}

    /**
     * Gets the pose of the robot.
     * 
     * @return A Pose2d representing the robot's position.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(getAngle(), getPositions(), newPose);
    }

    public DifferentialDriveWheelPositions getPositions() {
        return new DifferentialDriveWheelPositions(leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity()));
    }

    /**
     * Drives the robot at the desired ChassisSpeeds.
     * 
     * @param speeds The ChassisSpeeds object to drive at.
     */
    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds diffSpeeds = kinematics.toWheelSpeeds(speeds);
        frontLeftMotor.set(diffSpeeds.leftMetersPerSecond * DriveConstants.maxSpeed);
        frontRightMotor.set(diffSpeeds.rightMetersPerSecond * DriveConstants.maxSpeed);
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