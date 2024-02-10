package frc.robot.Drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    // Swerve Modules
    private final SwerveModule frontLeft = new SwerveModule("Front Left", DriveConstants.frontLeftDriveID, DriveConstants.frontLeftTurnID, DriveConstants.frontLeftCancoderID, DriveConstants.frontLeftOffset);
    private final SwerveModule frontRight = new SwerveModule("Front Right", DriveConstants.frontRightDriveID, DriveConstants.frontRightTurnID, DriveConstants.frontRightCancoderID, DriveConstants.frontRightOffset);
    private final SwerveModule backLeft = new SwerveModule("Back Left", DriveConstants.backLeftDriveID, DriveConstants.backLeftTurnID, DriveConstants.backLeftCancoderID, DriveConstants.backLeftOffset);
    private final SwerveModule backRight = new SwerveModule("Back Right", DriveConstants.backRightDriveID, DriveConstants.backRightTurnID, DriveConstants.backRightCancoderID, DriveConstants.backRightOffset);

    // Module Management
    private SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
    private SwerveModuleState[] states = new SwerveModuleState[4];
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    // Kinematics
    private ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(-DriveConstants.kModuleToCenter, DriveConstants.kModuleToCenter),
        new Translation2d(DriveConstants.kModuleToCenter, DriveConstants.kModuleToCenter),
        new Translation2d(-DriveConstants.kModuleToCenter, -DriveConstants.kModuleToCenter),
        new Translation2d(DriveConstants.kModuleToCenter, -DriveConstants.kModuleToCenter)
    );

    // Odometry
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getAngle(), modulePositions);

    // Gyro
    private Pigeon2 gyro = new Pigeon2(DriveConstants.gyroID);

    public Drivetrain() {
        for(SwerveModule module : modules) {
            module.initEncoder();
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
     * @return A rotation2d representing the angle of the robot.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    /**
     * Drives the drivetrain
     * @param sp The desired ChassisSpeeds
     */
    public void drive(ChassisSpeeds sp) {
        // System.out.println(sp.toString());
        // System.out.println(getRobotAngle().toString());
        this.speeds = sp;
        states = kinematics.toSwerveModuleStates(speeds, new Translation2d(0.0, 0.0));
        // modules[0].setState(states[0]);
        for (int i = 0; i < 4; i++) {
            modules[i].setState(states[i]);
            //modules[i].setState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)));
        }
    }

    /**
     * Stops all drivetrain movement
     */
    public void stop() {
        this.drive(
            new ChassisSpeeds(0.0, 0.0, 0.0)
        );
    }
}