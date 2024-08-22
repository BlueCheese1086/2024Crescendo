package frc.robot.Drivetrain;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import java.io.File;
import java.io.IOException;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drivetrain extends SubsystemBase {
    private SwerveDrive swerveDrive;

    private SwerveModuleState[] xStates = {
        new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)),
        new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)),
        new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)),
        new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)),
    };

    private Field2d field = new Field2d();
    private AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // A common instance of the drivetrain subsystem.
    private static Drivetrain instance;

    /**
     * This function gets a common instance of the drivetrain subsystem that anyone
     * can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and
     * instead run this function whenever we need the subsystem.
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
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(DriveConstants.maxDriveSpeed);
        } catch(IOException err) {
            throw new RuntimeException("Could not find SwerveDrive configs.");
        }

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        setupPathplanner();
    }

    public Drivetrain(SwerveDriveConfiguration driveConfig, SwerveControllerConfiguration controllerConfig) {
        swerveDrive = new SwerveDrive(driveConfig, controllerConfig, DriveConstants.maxDriveSpeed);
    }

    public void setupPathplanner() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getSpeeds,
            this::setSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD),
                new PIDConstants(DriveConstants.turnP, DriveConstants.turnI, DriveConstants.turnD),
                DriveConstants.maxDriveSpeed,
                DriveConstants.width / 2,
                new ReplanningConfig()
            ),
            () -> DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false,
            this
        );
    }

    @Override
    public void periodic() {
        // Updating the states and positions of the modules
        field.setRobotPose(getPose());

        SmartDashboard.putData("Field", field);
    }

    public Rotation2d getAngle() {
        return swerveDrive.getYaw();
    }

    public void setAngle(Rotation2d angle) {
        swerveDrive.setGyro(new Rotation3d(0, 0, angle.getRadians()));
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void setPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        swerveDrive.setChassisSpeeds(speeds);
    }

    public void makeX() {
        swerveDrive.setModuleStates(xStates, false);
    }

    public void playSong(String filepath) {
        Orchestra music = new Orchestra();

        // music.addInstrument(flModule.drive);
        // music.addInstrument(frModule.drive);
        // music.addInstrument(blModule.drive);
        // music.addInstrument(brModule.drive);
        music.loadMusic(filepath);
        
        music.play();
    }
}