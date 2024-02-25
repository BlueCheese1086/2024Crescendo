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

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    enum InitPoses {
        BLUE(new Pose2d(2, 6, new Rotation2d())),
        RED(new Pose2d(14.592, 6, new Rotation2d(Math.PI)));

        public Pose2d pose;

        InitPoses(Pose2d pose) {
            this.pose = pose;
        }
    }
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
    private final DifferentialDrivePoseEstimator estimator;
    private FieldObject2d fieldObj = field.getObject("Odometry");

    /** Creates a new instance of the Drivetrain subsystem. */
    public Drivetrain() {
        // Initializing the motors
        frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftID, MotorType.kBrushless);
        backLeftMotor = new CANSparkMax(DriveConstants.backLeftID, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(DriveConstants.frontRightID, MotorType.kBrushless);
        backRightMotor = new CANSparkMax(DriveConstants.backRightID, MotorType.kBrushless);

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
        leftEncoder.setPositionConversionFactor(DriveConstants.posConversionFactor);
        leftEncoder.setVelocityConversionFactor(DriveConstants.velConversionFactor);

        // Setting right encoder conversion values
        rightEncoder.setPositionConversionFactor(DriveConstants.posConversionFactor);
        rightEncoder.setVelocityConversionFactor(DriveConstants.velConversionFactor);

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
        kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);

        // Initializing Odometry
        initPose = new Pose2d(2, 6, new Rotation2d());
        pose = initPose;
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getYaw()), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
        estimator = new DifferentialDrivePoseEstimator(kinematics, getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition(), initPose);
        // odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
        // maybe use getAngle when creating initPose

        // Implementing PathPlanner
        AutoBuilder.configureRamsete(
            this::getPose,
            this::resetPose,
            this::getSpeeds,
            this::drive,
            0.25, // Straight PID P
            0.5, // Turn PID P
            new ReplanningConfig(),
            () -> DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red),
            // Watch initpose when alliance is red.  robot may go to the wrong side of the field.
            // NOTE: Driverstation defaults to red.
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

        Pose2d estimatedPose = estimator.update(getAngle(), getPositions());
        field.setRobotPose(estimatedPose);

        Pose2d odometryPose = odometry.update(getAngle(), getPositions());
        fieldObj.setPose(odometryPose);

        SmartDashboard.putData("Field", field);
    }

    /**
     * Gets the pose of the robot.
     * 
     * @return A Pose2d representing the robot's position.
     */
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry of the robot to the new pose.
     * 
     * @param newPose The new position of the robot.
     */
    public void resetPose(Pose2d newPose) {
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
        // Set out yardstick and test P value.
        // If robot overshoots the 1 meter mark, lower p.  reverse if undershot
        // add d if there is a minor overshoot.
        // add i for "smoothness", probably shouldn't use though
        // tune on carpet
        // be careful for canter
        // Do the same test for angular PID
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