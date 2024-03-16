package frc.robot.Drivetrain;

import com.ctre.phoenix.sensors.Pigeon2;

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
    private SwerveModule flModule = new SwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID, DriveConstants.flCancoderID, DriveConstants.flOffset);
    private SwerveModule frModule = new SwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID, DriveConstants.flCancoderID, DriveConstants.flOffset);
    private SwerveModule blModule = new SwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID, DriveConstants.flCancoderID, DriveConstants.flOffset);
    private SwerveModule brModule = new SwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID, DriveConstants.flCancoderID, DriveConstants.flOffset);

    // Sensors
    private Pigeon2 gyro = new Pigeon2(DriveConstants.gyroID);

    // Kinematics
    /*
     *         Y
     *
     *         |      
     *     FL  |  FR  
     *         |      
     * X -------------
     *         |      
     *     BL  |  BR  
     *         |      
     */
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(-DriveConstants.width / 2,  DriveConstants.length / 2), // FL Swerve module
        new Translation2d( DriveConstants.width / 2,  DriveConstants.length / 2), // FR Swerve module
        new Translation2d(-DriveConstants.width / 2, -DriveConstants.length / 2), // BL Swerve module
        new Translation2d( DriveConstants.width / 2, -DriveConstants.length / 2)  // BR Swerve module
    );

    // Swerve Module Vars
    private SwerveModule[] modules = new SwerveModule[4];
    private SwerveModuleState[] states = new SwerveModuleState[4];
    private SwerveModulePosition[] positions = new SwerveModulePosition[4];

    // Odometry
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getAngle(), positions);

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
            positions[i] = modules[i].getPosition();
        }

        odometry.update(getAngle(), positions);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        flModule.setState(states[0]);
        frModule.setState(states[1]);
        blModule.setState(states[2]);
        brModule.setState(states[3]);
    }
}
