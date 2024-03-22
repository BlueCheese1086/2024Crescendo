package frc.robot.Drivetrain;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dynamic;
import frc.robot.Constants.DriveConstants;
import frc.robot.Vision.Vision;

public class Drivetrain extends SubsystemBase {
    // Swerve Modules
    private SwerveModule flModule;
    private SwerveModule frModule;
    private SwerveModule blModule;
    private SwerveModule brModule;

    // Sensors
    private Pigeon2 gyro;

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
    private SwerveDriveKinematics kinematics;

    // Swerve Module Vars
    private SwerveModule[] modules = {flModule, frModule, blModule, brModule};
    private SwerveModulePosition[] positions = new SwerveModulePosition[4];

    // Odometry/Pose Estimation
    private SwerveDrivePoseEstimator poseEstimator;
    private Vision vision;

    public Drivetrain(Vision vision) {
        // Initializing the Swerve Modules
        flModule = new SwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID, DriveConstants.flCancoderID, DriveConstants.flOffset);
        frModule = new SwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID, DriveConstants.flCancoderID, DriveConstants.flOffset);
        blModule = new SwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID, DriveConstants.flCancoderID, DriveConstants.flOffset);
        brModule = new SwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID, DriveConstants.flCancoderID, DriveConstants.flOffset);

        // Initializing the gyro
        gyro = new Pigeon2(DriveConstants.gyroID);

        // Loading the initial values into the state and position arrays.
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }

        // Initializing the kinematics
        kinematics = new SwerveDriveKinematics(
            new Translation2d(-DriveConstants.width / 2,  DriveConstants.length / 2), // FL Swerve module
            new Translation2d( DriveConstants.width / 2,  DriveConstants.length / 2), // FR Swerve module
            new Translation2d(-DriveConstants.width / 2, -DriveConstants.length / 2), // BL Swerve module
            new Translation2d( DriveConstants.width / 2, -DriveConstants.length / 2)  // BR Swerve module
        );

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getAngle(), positions, new Pose2d());
        this.vision = vision;
    }

    @Override
    public void periodic() {
        // Updating the states and positions of the modules
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        
        // Updating the pose estimator
        poseEstimator.addVisionMeasurement(vision.getFrontPose(), Timer.getFPGATimestamp());
        poseEstimator.addVisionMeasurement(vision.getBackPose(), Timer.getFPGATimestamp());
        poseEstimator.update(getAngle(), positions);

        Dynamic.robotPose = poseEstimator.getEstimatedPosition();
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