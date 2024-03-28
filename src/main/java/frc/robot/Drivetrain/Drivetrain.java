package frc.robot.Drivetrain;

import java.util.Objects;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

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
    private SparkMaxSwerveModule flModule;
    private SparkMaxSwerveModule frModule;
    private SparkMaxSwerveModule blModule;
    private SparkMaxSwerveModule brModule;

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
    private SparkMaxSwerveModule[] modules = {flModule, frModule, blModule, brModule};
    private SwerveModulePosition[] positions = new SwerveModulePosition[4];

    // Odometry/Pose Estimation
    private SwerveDrivePoseEstimator poseEstimator;
    private Vision vision;

    // A common instance of the drivetrain subsystem.
    private static Drivetrain instance;

    public Drivetrain() {
        // Initializing the Swerve Modules
        flModule = new SparkMaxSwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID, DriveConstants.flCancoderID, DriveConstants.flOffset);
        frModule = new SparkMaxSwerveModule(DriveConstants.frDriveID, DriveConstants.frTurnID, DriveConstants.frCancoderID, DriveConstants.frOffset);
        blModule = new SparkMaxSwerveModule(DriveConstants.blDriveID, DriveConstants.blTurnID, DriveConstants.blCancoderID, DriveConstants.blOffset);
        brModule = new SparkMaxSwerveModule(DriveConstants.brDriveID, DriveConstants.brTurnID, DriveConstants.brCancoderID, DriveConstants.brOffset);

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

        // Initializing the pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getAngle(), positions, new Pose2d());

        // Getting an instance of the Vision subsystem.
        vision = Vision.getInstance();
    }

    /**
     * This function gets a common instance of the drivetrain subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Drivetrain subsystem.
     */
    public static Drivetrain getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (Objects.isNull(instance)) instance = new Drivetrain();

        return instance;
    }

    @Override
    public void periodic() {
        // Updating the states and positions of the modules
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        
        // Getting the estimated pose from the vision system
        Optional<EstimatedRobotPose> estimatedPose = vision.getEstimatedPose(poseEstimator.getEstimatedPosition());

        // If there is a pose from the vision system, then this adds it to the current pose estimator
        if (estimatedPose.isPresent()) {
            poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
        }

        // Updating the pose estimator
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