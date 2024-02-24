package frc.robot.Drivetrain;

import com.ctre.phoenix.sensors.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    // Swerve Modules
    private SwerveModule frontLeft  = new SwerveModule("Front Left",  DriveConstants.flDriveID, DriveConstants.flTurnID, DriveConstants.flCancoderID, DriveConstants.flOffset);
    private SwerveModule frontRight = new SwerveModule("Front Right", DriveConstants.frDriveID, DriveConstants.frTurnID, DriveConstants.frCancoderID, DriveConstants.frOffset);
    private SwerveModule backLeft   = new SwerveModule("Back Left",   DriveConstants.blDriveID, DriveConstants.blTurnID, DriveConstants.blCancoderID, DriveConstants.blOffset);
    private SwerveModule backRight  = new SwerveModule("Back Right",  DriveConstants.brDriveID, DriveConstants.brTurnID, DriveConstants.brCancoderID, DriveConstants.brOffset);

    // Gyro
    private Pigeon2 gyro = new Pigeon2(DriveConstants.gyroID);

    // Module Management
    private SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
    private SwerveModuleState[] states = {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    private SwerveModulePosition[] positions = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};

    // Kinematics
    private ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.flPosition, DriveConstants.frPosition, DriveConstants.blPosition, DriveConstants.brPosition);

    // Odometry
    private Pose2d pose = new Pose2d(1.23, 6.25, Rotation2d.fromDegrees(32.62));
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getAngle(), positions, pose);
    private Field2d field = new Field2d();

    public Drivetrain() {
        // Resetting the angle of the gyro.
        gyro.setYaw(0);
        

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
            () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
            this
        );
    }

    /**
     * Runs every tick (20 ms)
     */
    @Override
    public void periodic() {
        // Updating the States and Positions of each swerve module
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
            speeds = kinematics.toChassisSpeeds(states);
        }

        // Updating odometry
        pose = odometry.update(getAngle(), new SwerveDriveWheelPositions(positions));

        // Updating the Field2d element for Glass
        field.setRobotPose(pose);
        SmartDashboard.putData("Field2d", field);
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
     * <p>
     * Mainly used for PathPlanner configuration.
     * 
     * @return An estimated pose of the robot.
     */
    public Pose2d getPose() {
        return this.pose;
    }

    /**
     * Resets the odometry.
     * <p>
     * Mainly used for PathPlanner configuration.
     * 
     * @param newPose The new position of the robot.
     */
    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(getAngle(), positions, newPose);
    }

    /**
     * Gets the speeds of the robot.
     * <p>
     * Mainly used for PathPlanner configuration.
     * 
     * @return the speeds of the robot.
     */
    public ChassisSpeeds getSpeeds() {
        return this.speeds;
    }

    /**
     * Drives the drivetrain.
     * 
     * @param speeds The desired ChassisSpeeds
     */
    public void drive(ChassisSpeeds speeds) {
        // Converting ChassisSpeeds to SwerveModuleStates.
        states = kinematics.toSwerveModuleStates(speeds);

        SmartDashboard.putNumber("Angle from ChassisSpeeds", speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Angle from kinematics", states[1].angle.getDegrees());

        // Setting the state of each swerve module.
        for (int i = 0; i < 4; i++) {
            modules[i].setState(states[i]);
        }
    }
}