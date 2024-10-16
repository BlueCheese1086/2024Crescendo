package frc.robot.Drivetrain;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Vision.Vision;
import org.photonvision.EstimatedRobotPose;

public class Drivetrain extends SubsystemBase {
    // Swerve Modules
    private SwerveModule flModule;
    private SwerveModule frModule;
    private SwerveModule blModule;
    private SwerveModule brModule;

    // Sensors
    private Pigeon2 gyro;

    // Kinematics
    private SwerveDriveKinematics kinematics;

    // Swerve Module Vars
    private SwerveModule[] modules;
    private SwerveModulePosition[] positions = new SwerveModulePosition[4];
    private SwerveModuleState[] states = new SwerveModuleState[4];
    private SwerveModuleState[] xStates = {
        new SwerveModuleState(0.0, new Rotation2d( Math.PI / 4.0)),
        new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)),
        new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)),
        new SwerveModuleState(0.0, new Rotation2d( Math.PI / 4.0)),
    };

    // Odometry/Pose Estimation
    private Field2d field = new Field2d();
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDriveOdometry odometry;

    // An instance of the Vision class
    private Vision vision;

    // A common instance of the drivetrain subsystem.
    private static Drivetrain instance;

    /**
     * This function gets a common instance of the drivetrain subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Drivetrain subsystem.
     */
    public static Drivetrain getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (instance == null)
            instance = new Drivetrain();

        return instance;
    }

    public Drivetrain() {
        // Initializing the Swerve Modules
        flModule = new SwerveModule("FrontLeft", DriveConstants.flDriveID, DriveConstants.flTurnID,
                DriveConstants.flCancoderID, DriveConstants.flOffset);
        frModule = new SwerveModule("FrontRight", DriveConstants.frDriveID, DriveConstants.frTurnID,
                DriveConstants.frCancoderID, DriveConstants.frOffset);
        blModule = new SwerveModule("BackLeft", DriveConstants.blDriveID, DriveConstants.blTurnID,
                DriveConstants.blCancoderID, DriveConstants.blOffset);
        brModule = new SwerveModule("BackRight", DriveConstants.brDriveID, DriveConstants.brTurnID,
                DriveConstants.brCancoderID, DriveConstants.brOffset);

        // Loading the modules array
        modules = new SwerveModule[] {flModule, frModule, blModule, brModule};

        // Initializing the gyro
        gyro = new Pigeon2(DriveConstants.gyroID);

        // Loading the initial values into the state and position arrays.
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }

        // Initializing the kinematics
        kinematics = new SwerveDriveKinematics(
            new Translation2d( DriveConstants.width / 2,  DriveConstants.length / 2), // FL Swerve module
            new Translation2d( DriveConstants.width / 2, -DriveConstants.length / 2), // FR Swerve module
            new Translation2d(-DriveConstants.width / 2,  DriveConstants.length / 2), // BL Swerve module
            new Translation2d(-DriveConstants.width / 2, -DriveConstants.length / 2)  // BR Swerve module
        );

        // Initializing the pose estimator
        odometry = new SwerveDriveOdometry(kinematics, getAngle(), positions);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getAngle(), positions, new Pose2d());

        // Getting an instance of the Vision system.
        vision = Vision.getInstance();
    }

    @Override
    public void periodic() {
        // Updating the states and positions of the modules
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
            states[i] = modules[i].getState();
        }

        // Updating the pose estimator
        poseEstimator.update(getAngle(), positions);
        
        EstimatedRobotPose lResult = vision.getLPose();
        if (lResult != null) poseEstimator.addVisionMeasurement(lResult.estimatedPose.toPose2d(), lResult.timestampSeconds);

        EstimatedRobotPose rResult = vision.getRPose();
        if (rResult != null) poseEstimator.addVisionMeasurement(rResult.estimatedPose.toPose2d(), rResult.timestampSeconds);

        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("Gyro", gyro.getAngle());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void setAngle(Rotation2d angle) {
        gyro.setYaw(angle.getDegrees());
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < 4; i++) {
            modules[i].setState(desiredStates[i]);
        }
    }

    public void makeX() {
        for (int i = 0; i < 4; i++) {
            modules[i].setState(xStates[i]);
        }
    }

    public void playSong(String filepath) {
        // Creating the orchestra object.
        Orchestra music = new Orchestra();

        // Making every motor part of the orchestra
        music.addInstrument(flModule.drive);
        music.addInstrument(frModule.drive);
        music.addInstrument(blModule.drive);
        music.addInstrument(brModule.drive);

        // Letting the driver know that music is playing.
        SmartDashboard.putBoolean("/PlayingMusic", true);

        // Finding the music
        music.loadMusic(filepath);

        // Playing the music
        music.play();

        // Preventing the robot from moving while music is playing.
        while (music.isPlaying()) {
            makeX();
        }

        // Letting the driver know that music is done playing.
        SmartDashboard.putBoolean("/PlayingMusic", false);
        
        // Ending the orchestra.
        music.close();
    }
}