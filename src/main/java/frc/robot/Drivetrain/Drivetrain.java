package frc.robot.Drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    // Getting pids for the primary (front) motors.
    private SparkPIDController leftPID;
    private SparkPIDController rightPID;

    // Gyro
    private PigeonIMU gyro;

    // Kinematics
    public final DifferentialDriveKinematics kinematics;

    // Odometry
    public final Pose2d initPose;
    private Pose2d pose;
    private Field2d field = new Field2d();
    private final DifferentialDriveOdometry odometry;

    /** Creates a new instance of the Drivetrain subsystem. */
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

        // Initializing the encoders
        leftEncoder = frontLeftMotor.getEncoder();
        rightEncoder = frontRightMotor.getEncoder();

        // Resetting encoders
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        // Setting left encoder conversion values
        leftEncoder.setPositionConversionFactor(DriveConstants.wheelCircumference / DriveConstants.gearRatio);
        leftEncoder.setVelocityConversionFactor(DriveConstants.wheelCircumference / DriveConstants.gearRatio / 60);

        // Setting right encoder conversion values
        rightEncoder.setPositionConversionFactor(DriveConstants.wheelCircumference / DriveConstants.gearRatio);
        rightEncoder.setVelocityConversionFactor(DriveConstants.wheelCircumference / DriveConstants.gearRatio / 60);

        // Initializing the PIDs
        leftPID = frontLeftMotor.getPIDController();
        rightPID = frontRightMotor.getPIDController();

        // Setting values for the left PID
        leftPID.setP(0.25);
        leftPID.setI(0);
        leftPID.setD(0);
        leftPID.setFF(0);

        // Setting values for the right PID
        rightPID.setP(0.25);
        rightPID.setI(0);
        rightPID.setD(0);
        rightPID.setFF(0);

        // Saving the sparkmax settings
        frontLeftMotor.burnFlash();
        backLeftMotor.burnFlash();
        frontRightMotor.burnFlash();
        backRightMotor.burnFlash();

        // Initializing Gyro
        gyro = new PigeonIMU(DriveConstants.gyroID);
        gyro.setYaw(0);

        // Initializing Kinematics
        kinematics = new DifferentialDriveKinematics(DriveConstants.kModuleToModuleDistance);

        // Initializing Odometry
        initPose = new Pose2d(2, 7, new Rotation2d());
        pose = initPose;
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getYaw()), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);

        // Implementing PathPlanner
        AutoBuilder.configureRamsete(
            this::getPose,
            this::resetPose,
            this::getSpeeds,
            this::drive,
            new ReplanningConfig(),
            () -> DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red),
            this
        );
    }

    /** 
     * This function runs every tick.
     * Right now, I am only using it to update the state of the odometry.
    */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Angle", getAngle().getDegrees());
        this.pose = odometry.update(getAngle(), getPositions());
        field.setRobotPose(pose);
        SmartDashboard.putData("Field", field);
    }

    /**
     * Gets the pose of the robot.
     * 
     * @return A Pose2d representing the robot's position.
     */
    public Pose2d getPose() {
        return this.pose;
    }

    /**
     * Resets the odometry of the robot to the new pose.
     * 
     * @param newPose The new position of the robot.
     */
    public void resetPose(Pose2d newPose) {
        // Resetting encoders
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        // Resetting gyro
        gyro.setYaw(0);

        // Resetting position
        odometry.resetPosition(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition(), newPose);
    }

    /**
     * Drives the robot at the desired ChassisSpeeds.
     * 
     * @param speeds The ChassisSpeeds object to drive at.
     */
    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds diffSpeeds = kinematics.toWheelSpeeds(speeds);
        SmartDashboard.putNumber("Left Speed", diffSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Right Speed", diffSpeeds.rightMetersPerSecond);

        leftPID.setReference(diffSpeeds.leftMetersPerSecond, ControlType.kVelocity);
        rightPID.setReference(diffSpeeds.rightMetersPerSecond, ControlType.kVelocity);
    }

    /**
     * Returns the current angle of the robot.
     * 
     * @return The angle of the robot as a Rotation2D
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * Gets the positions of each motor.
     * 
     * @return The positions of each motor.
     */
    public DifferentialDriveWheelPositions getPositions() {
        return new DifferentialDriveWheelPositions(leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    /**
     * Gets the speeds of the robot.
     * 
     * @return The speeds that the robot is moving at.
     */
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity()));
    }
}