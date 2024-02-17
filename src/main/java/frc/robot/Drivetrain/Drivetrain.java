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
    private SwerveModule frontLeft;
    private SwerveModule backLeft;
    private SwerveModule frontRight;
    private SwerveModule backRight;

    // Module Management
    private SwerveModule[] modules = new SwerveModule[4];
    private SwerveModuleState[] states = new SwerveModuleState[4];

    // Kinematics
    private final SwerveDriveKinematics kinematics;

    // Gyro
    private Pigeon2 gyro;

    public Drivetrain() {
        // Initializing Swerve Modules
        frontLeft = new SwerveModule(
            "Front Left",
            DriveConstants.frontLeftDriveID,
            DriveConstants.frontLeftTurnID,
            DriveConstants.frontLeftCancoderID,
            DriveConstants.frontLeftOffset
        );

        backLeft = new SwerveModule(
            "Back Left",
            DriveConstants.backLeftDriveID,
            DriveConstants.backLeftTurnID,
            DriveConstants.backLeftCancoderID,
            DriveConstants.backLeftOffset
        );
        
        frontRight = new SwerveModule(
            "Front Right",
            DriveConstants.frontRightDriveID,
            DriveConstants.frontRightTurnID,
            DriveConstants.frontRightCancoderID,
            DriveConstants.frontRightOffset
        );

        backRight = new SwerveModule(
            "Back Right",
            DriveConstants.backRightDriveID,
            DriveConstants.backRightTurnID,
            DriveConstants.backRightCancoderID,
            DriveConstants.backRightOffset
        );

        // Initializing Gyro
        gyro = new Pigeon2(DriveConstants.gyroID);
        gyro.setYaw(0);
        
        // Initializing kinematics
        modules = new SwerveModule[]{frontLeft, backLeft, frontRight, backRight};
        kinematics = new SwerveDriveKinematics(
            new Translation2d(DriveConstants.kModuleToCenter, DriveConstants.kModuleToCenter),  // Front Left
            new Translation2d(-DriveConstants.kModuleToCenter, DriveConstants.kModuleToCenter), // Back Left
            new Translation2d(DriveConstants.kModuleToCenter, -DriveConstants.kModuleToCenter), // Front Right
            new Translation2d(-DriveConstants.kModuleToCenter, -DriveConstants.kModuleToCenter) // Back Right
        );

        // Initializing the turn encoders on each swerve module. 
        for (int i = 0; i < modules.length; i++) {
            modules[i].initializeEncoder();
        }
    }

    /**
     * Runs every tick (20 ms)
     */
    @Override
    public void periodic() {}

    /**
     * Gets the angle the robot is facing.
     * 
     * @return A Rotation2d representing the angle of the robot.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * Gets the speeds of each swerve module
     * 
     * @return A ChassisSpeeds representing the speeds of each swerve module.
     */
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(states);
    }

    /**
     * Drives the drivetrain
     * 
     * @param sp The desired ChassisSpeeds
     */
    public void drive(ChassisSpeeds speeds) {
        states = kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < 4; i++) {
            modules[i].setState(states[i]);
        }
    }

    /** Stops all drivetrain movement. */
    public void stop() {
        for (SwerveModule m : modules) {
            m.stop();
        }
    }
}