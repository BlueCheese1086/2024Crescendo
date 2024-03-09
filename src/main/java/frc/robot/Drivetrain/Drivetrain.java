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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    // Setting the initial poses for the robot.
    enum InitPoses {
        BLUE(new Pose2d(2, 7, new Rotation2d())),
        //16.592 m
        RED(new Pose2d(14.592, 7, new Rotation2d(Math.PI)));

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

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = MutableMeasure.mutable(Units.Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = MutableMeasure.mutable(Units.MetersPerSecond.of(0));

    // Gyro
    private PigeonIMU gyro;

    // Kinematics
    public final DifferentialDriveKinematics kinematics;

    // Odometry
    private Pose2d pose;
    private Field2d field = new Field2d();
    private final DifferentialDriveOdometry odometry;

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
        leftPID.setP(0.5);
        leftPID.setI(0);
        leftPID.setD(0);
        leftPID.setFF(0.5);

        // Setting values for the right PID
        rightPID.setP(0.5);
        rightPID.setI(0);
        rightPID.setD(0);
        rightPID.setFF(0.5);

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

        // Getting alliance
        boolean isRed = DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

        // Initializing pose
        // Try not doing this.  Maybe the autobuilder will initialize it for me.
        pose = isRed ? InitPoses.RED.pose : InitPoses.BLUE.pose;
        
        // Initializing Odometry
        odometry = new DifferentialDriveOdometry(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);

        // Implementing PathPlanner
        AutoBuilder.configureRamsete(
            this::getPose,
            this::resetPose,
            this::getSpeeds,
            this::drive,
            1, // Forward PID P.
            0.4, // Turn PID P.
            new ReplanningConfig(),
            () -> isRed,
            this
        );

        // Last test results:
        // b: 0.25
        // distance moved: 0.43m
        // target distance: 1m
        // To do: Increase b
        /*
         * 0.5 undershot by 9.5 in
         * 0.75 overshot by 4.5 in
         * 
         * 0.25 b = 14 in
         * 0.25/14 b = 1 in
         * 
         * 0.64286 b = 36 in
         */

        // Set out yardstick and test P value.
        // If robot overshoots the 1 meter mark, lower p.  reverse if undershot
        // add d if there is a minor overshoot. (don't need though.)
        // add i for "smoothness", probably shouldn't use though
        // tune on carpet
        // be careful for canter
        // Do the same test for angular PID
    }

    private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                frontLeftMotor.setVoltage(volts.in(Units.Volts));
                frontRightMotor.setVoltage(volts.in(Units.Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .linearPosition(
                        m_distance.mut_replace(leftEncoder.getPosition(), Units.Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(leftEncoder.getVelocity(), Units.MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .linearPosition(
                        m_distance.mut_replace(rightEncoder.getPosition(), Units.Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(rightEncoder.getVelocity(), Units.MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

    public Command sysIdDynamic(Direction dir) {
        return sysIdRoutine.dynamic(dir);
    }

    public Command sysIdQuasistatic(Direction dir) {
        return sysIdRoutine.quasistatic(dir);
    }

    /** 
     * This function runs every tick.
     * Right now, I am only using it to update the state of the odometry.
    */
    @Override
    public void periodic() {        
        pose = odometry.update(getAngle(), getPositions());
        field.setRobotPose(pose);

        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
        SmartDashboard.putBoolean("At 1m", (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2 > 1);
        SmartDashboard.putNumber("Left Speed", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Speed", rightEncoder.getVelocity());
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