package frc.robot.Drivetrain;

import com.ctre.phoenix.sensors.Pigeon2;

import Util.IntializedSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase implements IntializedSubsystem {

    /*           backLeft    frontLeft
     *                 x_____x
     *                 |     |
     * --------shooter-----------intake--- x-axis
     *                 |_____|
     *                 x     x
     *           backRight   frontRight
     */

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] modules;

    private final Pigeon2 gyro;

    public Drivetrain() {
        frontLeft = new SwerveModule(
            "FrontLeft",
            DriveConstants.frontLeftDriveID,
            DriveConstants.frontLeftTurnID, 
            DriveConstants.frontLeftEncID, 
            DriveConstants.frontLeftOffset);

        backLeft = new SwerveModule(
            "BackLeft",
            DriveConstants.backLeftDriveID,
            DriveConstants.backLeftTurnID, 
            DriveConstants.backLeftEncID, 
            DriveConstants.backLeftOffset);

        frontRight = new SwerveModule(
            "FrontRight",
            DriveConstants.frontRightDriveID,
            DriveConstants.frontRightTurnID, 
            DriveConstants.frontRightEncID, 
            DriveConstants.frontRightOffset);

        backRight = new SwerveModule(
            "BackRight",
            DriveConstants.backRightDriveID,
            DriveConstants.backRightTurnID, 
            DriveConstants.backRightEncID, 
            DriveConstants.backRightOffset);

        modules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        kinematics = new SwerveDriveKinematics(
            new Translation2d(DriveConstants.moduleToModuleDistanceMeters/2.0, DriveConstants.moduleToModuleDistanceMeters/2.0),
            new Translation2d(DriveConstants.moduleToModuleDistanceMeters/2.0, -DriveConstants.moduleToModuleDistanceMeters/2.0),
            new Translation2d(-DriveConstants.moduleToModuleDistanceMeters/2.0, DriveConstants.moduleToModuleDistanceMeters/2.0),
            new Translation2d(-DriveConstants.moduleToModuleDistanceMeters/2.0, -DriveConstants.moduleToModuleDistanceMeters/2.0)
        );

        gyro = new Pigeon2(DriveConstants.pigeonID);

    }

    public void initialize() {
        for (SwerveModule m : modules) {
            m.initializeEncoder();
        }
    }

    public void periodic() {}

    public void initPigeon() {
        gyro.setYaw(0.0);
        // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        //     gyro.setYaw(0.0);
        // } else {
        //     gyro.setYaw(180.0);
        // }
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public void stop() {
        for (SwerveModule m : modules) {
            m.stop();
        }
    }
    
}
