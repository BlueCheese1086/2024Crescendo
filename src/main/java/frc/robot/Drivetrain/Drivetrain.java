package frc.robot.Drivetrain;

import com.ctre.phoenix.sensors.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    // Swerve Modules
    private SwerveModule frontLeft  = new SwerveModule("Front Left",  DriveConstants.frontLeftDriveID,  DriveConstants.frontLeftTurnID,  DriveConstants.frontLeftCancoderID,  DriveConstants.frontLeftOffset );
    private SwerveModule backLeft   = new SwerveModule("Back Left",   DriveConstants.backLeftDriveID,   DriveConstants.backLeftTurnID,   DriveConstants.backLeftCancoderID,   DriveConstants.backLeftOffset  );
    private SwerveModule frontRight = new SwerveModule("Front Right", DriveConstants.frontRightDriveID, DriveConstants.frontRightTurnID, DriveConstants.frontRightCancoderID, DriveConstants.frontRightOffset);
    private SwerveModule backRight  = new SwerveModule("Back Right",  DriveConstants.backRightDriveID,  DriveConstants.backRightTurnID,  DriveConstants.backRightCancoderID,  DriveConstants.backRightOffset );

    // Module Management
    private SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
    private SwerveModuleState[] states = {frontLeft.state, frontRight.state, backLeft.state, backRight.state};
    private SwerveModulePosition[] positions = {frontLeft.position, frontRight.position, backLeft.position, backRight.position};

    // Kinematics
    private ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(DriveConstants.kModuleToCenter, DriveConstants.kModuleToCenter),
        new Translation2d(DriveConstants.kModuleToCenter, -DriveConstants.kModuleToCenter),
        new Translation2d(-DriveConstants.kModuleToCenter, DriveConstants.kModuleToCenter),
        new Translation2d(-DriveConstants.kModuleToCenter, -DriveConstants.kModuleToCenter)
    );

    // Odometry
    private final Pose2d initPose = new Pose2d(1.23, 6.25, Rotation2d.fromDegrees(32.62));
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getAngle(), positions, initPose);
    private Pose2d pose = initPose;

    // Gyro
    private Pigeon2 gyro = new Pigeon2(DriveConstants.gyroID);

    public Drivetrain() {
        // Initializing the turn encoders on each swerve module. 
        for (int i = 0; i < modules.length; i++) {
            modules[i].initEncoder();
        }

        // Creating an AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            this::getSpeeds,
            this::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD),
                new PIDConstants(DriveConstants.turnP, DriveConstants.turnI, DriveConstants.turnD),
                DriveConstants.maxDriveSpeed,
                DriveConstants.kModuleToCenter,
                new ReplanningConfig()
            ),
            () -> {
                return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false;
            },
            this);
    }

    /**
     * Runs every tick (20 ms)
     */
    @Override
    public void periodic() {
        // Updating the States and Positions of each swerve module
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].state;
            positions[i] = modules[i].position;
        }
    }

    /**
     * Gets the angle the robot is facing.
     * 
     * @return A rotation2d representing the angle of the robot.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * Sets the angle the robot is facing to a value.
     * <p>
     * Note that this doesn't actually rotate the robot to match the angle.
     * 
     * @param angle A rotation2d representing the new angle of the robot.
     */
    public void setAngle(Rotation2d angle) {
        gyro.setYaw(angle.getDegrees());
    }

    /**
     * Gets the estimated position of the robot.
     * 
     * @return An estimated pose of the robot
     */
    public Pose2d getPose() {
        return this.pose;
    }

    /** Resets the odometry. */
    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(getAngle(), positions, newPose);
    }

    public ChassisSpeeds getSpeeds() {
        return this.speeds;
    }

    /**
     * Drives the drivetrain
     * @param sp The desired ChassisSpeeds
     */
    public void drive(ChassisSpeeds speeds) {
        this.speeds = speeds;
        states = kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < 4; i++) {
            modules[i].setState(states[i]);
        }
    }

    /**
     * Stops all drivetrain movement
     */
    public void stop() {
        drive(
            new ChassisSpeeds(0.0, 0.0, 0.0)
        );
    }
}