package frc.robot.Drivetrain;

import java.util.Objects;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import Util.Interfaces.InitializedSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.SensorsAndFeedback.Gyro;

/*
                         __
                   _.--""  |
    .----.     _.-'   |/\| |.--.
    |1086|__.-'   _________|  |_)  _______________  
    |  .-""-.""""" ___,    `----'"))   __   .-""-.""""--._  
    '-' ,--. `   |blue|   .---.       |:.| ' ,--. `      _`.
     ( (    ) )__|cheese|__ \\|// _..--  \/ ( (    ) )--._".-.
      . `--' ;\__________________..--------. `--' ;--------'
       `-..-'                               `-..-'

*/

public class Drivetrain extends SubsystemBase implements InitializedSubsystem {

    /*
     * backLeft frontLeft
     * x_____x
     * | |
     * --------shooter-----------intake--- x-axis
     * |_____|
     * x x
     * backRight frontRight
     */

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] modules;
    private final Field2d field = new Field2d();

    private SwerveModuleState[] states = new SwerveModuleState[4];
    private SwerveModulePosition[] positions = new SwerveModulePosition[4];

    private SwerveModuleState[] xStates = new SwerveModuleState[] {
            new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)),
            new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)),
            new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)),
            new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)),
    };

    private final PIDController translationController = new PIDController(DriveConstants.translationControllerkP, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(DriveConstants.thetaControllerkP, 0.0, 0.0);

    private SwerveDriveOdometry odometry;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (Objects.isNull(instance))
            instance = new Drivetrain();
        return instance;
    }

    private final Gyro gyro;

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

        modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };
        kinematics = new SwerveDriveKinematics(
                new Translation2d(DriveConstants.moduleToModuleDistanceMeters / 2.0,
                        DriveConstants.moduleToModuleDistanceMeters / 2.0),
                new Translation2d(DriveConstants.moduleToModuleDistanceMeters / 2.0,
                        -DriveConstants.moduleToModuleDistanceMeters / 2.0),
                new Translation2d(-DriveConstants.moduleToModuleDistanceMeters / 2.0,
                        DriveConstants.moduleToModuleDistanceMeters / 2.0),
                new Translation2d(-DriveConstants.moduleToModuleDistanceMeters / 2.0,
                        -DriveConstants.moduleToModuleDistanceMeters / 2.0));

        gyro = Gyro.getInstance();

        for (int i = 0; i < positions.length; i++) {
            positions[i] = new SwerveModulePosition();
        }

        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), positions);

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getSpeeds,
                this::drive,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(DriveConstants.translationAutoControllerkP, 0, 0),
                        new PIDConstants(DriveConstants.thetaAutoControllerkP, 0, 0),
                        DriveConstants.maxWheelVelocity,
                        DriveConstants.moduleToCenterDistance,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                this);
    }

    /**
     * Initializes the subsystem: sets swerve states, intializes encoders and
     * odometry
     */
    public void initialize() {
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModuleState();
        }
        for (SwerveModule m : modules) {
            m.initializeEncoder();
        }
        gyro.initGyro();
        odometry = new SwerveDriveOdometry(kinematics, gyro.getAngle(), positions);
    }

    /**
     * Updates swerve module positions and odometry
     * Publishes telemetry
     */
    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
            states[i] = modules[i].getState();
        }

        odometry.update(gyro.getAngle(), positions);
        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putData(field);
    }

    /**
     * Drives the robot at the desired speeds with an overall feedback loop to
     * ensure the speeds are met
     * 
     * @param speeds The desired speeds of the drivetrain
     */
    public void drive(ChassisSpeeds speeds) {
        ChassisSpeeds currentSpeeds = getSpeeds();
        ChassisSpeeds newSpeeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond + translationController.calculate(currentSpeeds.vxMetersPerSecond, speeds.vxMetersPerSecond),// + 0.001 * (currentSpeeds.vxMetersPerSecond - speeds.vxMetersPerSecond),
                speeds.vyMetersPerSecond + translationController.calculate(currentSpeeds.vyMetersPerSecond, speeds.vyMetersPerSecond),// + 0.001 * (currentSpeeds.vyMetersPerSecond - speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond + thetaController.calculate(currentSpeeds.omegaRadiansPerSecond, speeds.omegaRadiansPerSecond));// + 0.0001 * (gyro.getYawVelocity().getRadians() - speeds.omegaRadiansPerSecond));
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(newSpeeds);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(desiredStates[i]);
        }
    }

    /**
     * Drives the robot at the desired speeds with an overall feedback loop to
     * ensure the speeds are met
     * 
     * @param speeds The desired speeds of the drivetrain
     */
    public void driveRated(ChassisSpeeds speeds) {
        ChassisSpeeds newSpeeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond,// + 0.001 * (currentSpeeds.vxMetersPerSecond - speeds.vxMetersPerSecond),
                speeds.vyMetersPerSecond,// + 0.001 * (currentSpeeds.vyMetersPerSecond - speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond);// + 0.0001 * (gyro.getYawVelocity().getRadians() - speeds.omegaRadiansPerSecond));
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(newSpeeds);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setRatedState(desiredStates[i]);
        }
    }

    /**
     * @return Returns total current drawn by the drive motors
     */
    public double getDriveCurrent() {
        int sum = 0;
        for (SwerveModule m : modules) {
            sum += m.getDriveCurrent();
        }
        return sum;
    }

    /**
     * @return Returns swerve module instances
     */
    public SwerveModule[] getModules() {
        return modules;
    }

    /**
     * @return Returns the robot position reported by the odometry
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * @return Returns all swerve module positions in order of [FL, FR, BL, BR]
     */
    public SwerveModulePosition[] getPositions() {
        return positions;
    }

    /**
     * @return Returns the chassis speeds reported by the measured module states
     */
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(states);
    }

    /**
     * @return Returns total current drawn by the turn motors
     */
    public double getTurnCurrent() {
        int sum = 0;
        for (SwerveModule m : modules) {
            sum += m.getTurnCurrent();
        }
        return sum;
    }

    /**
     * Sets the odometry position and updates the gyro angle
     * 
     * @param p The position to set odometry to
     */
    public void resetPose(Pose2d p) {
        field.setRobotPose(p);
        odometry.resetPosition(gyro.getAngle(), positions, p);
    }

    /**
     * Sets the drive PID FF value
     * @param FF The desired FF value
     */
    public void setDriveFF(double FF) {
        for (SwerveModule m : modules) {
            m.setDriveFF(FF);
        }
    }

    /**
     * Sets the modules in an X pattern
     */
    public void setX() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(xStates[i]);
        }
    }

    /**
     * Stops all modules
     */
    public void stop() {
        for (SwerveModule m : modules) {
            m.stop();
        }
    }

}