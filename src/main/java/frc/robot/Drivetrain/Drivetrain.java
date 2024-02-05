package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants.DriveConstants;

public class Drivetrain {

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

    public Drivetrain() {
        frontLeft = new SwerveModule(
            DriveConstants.frontLeftDriveID,
            DriveConstants.frontLeftTurnID, 
            DriveConstants.frontLeftEncID, 
            DriveConstants.frontLeftOffset);

        backLeft = new SwerveModule(
            DriveConstants.backLeftDriveID,
            DriveConstants.backLeftTurnID, 
            DriveConstants.backLeftEncID, 
            DriveConstants.backLeftOffset);

        frontRight = new SwerveModule(
            DriveConstants.frontRightDriveID,
            DriveConstants.frontRightTurnID, 
            DriveConstants.frontRightEncID, 
            DriveConstants.frontRightOffset);

        backRight = new SwerveModule(
            DriveConstants.backRightDriveID,
            DriveConstants.backRightTurnID, 
            DriveConstants.backRightEncID, 
            DriveConstants.backRightOffset);

        modules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        kinematics = new SwerveDriveKinematics(
            new Translation2d(1, 1),
            new Translation2d(1, -1),
            new Translation2d(-1, 1),
            new Translation2d(-1, -1)
        );

        for (SwerveModule m : modules) {
            m.initializeEncoder();
        }

    }
    
}
