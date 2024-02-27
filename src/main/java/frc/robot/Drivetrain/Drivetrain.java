package frc.robot.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import Util.IntializedSubsystem;
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
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Sensors.Gyro;

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
    private final Field2d field = new Field2d();

    private SwerveModuleState[] states = new SwerveModuleState[4];
    private SwerveModulePosition[] positions = new SwerveModulePosition[4];

    private SwerveModuleState[] xStates = new SwerveModuleState[]{
        new SwerveModuleState(0.0, new Rotation2d(Math.PI/4.0)),
        new SwerveModuleState(0.0, new Rotation2d(-Math.PI/4.0)),
        new SwerveModuleState(0.0, new Rotation2d(-Math.PI/4.0)),
        new SwerveModuleState(0.0, new Rotation2d(Math.PI/4.0)),
    };

    private SwerveDriveOdometry odometry;

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

        modules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        kinematics = new SwerveDriveKinematics(
            new Translation2d(DriveConstants.moduleToModuleDistanceMeters/2.0, DriveConstants.moduleToModuleDistanceMeters/2.0),
            new Translation2d(DriveConstants.moduleToModuleDistanceMeters/2.0, -DriveConstants.moduleToModuleDistanceMeters/2.0),
            new Translation2d(-DriveConstants.moduleToModuleDistanceMeters/2.0, DriveConstants.moduleToModuleDistanceMeters/2.0),
            new Translation2d(-DriveConstants.moduleToModuleDistanceMeters/2.0, -DriveConstants.moduleToModuleDistanceMeters/2.0)
        );

        gyro = Gyro.getInstance();

        for (int i = 0; i < positions.length; i++) {
            positions[i] = new SwerveModulePosition();
        }

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            this::getSpeeds,
            this::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(1.0, 0, 0), // try without these?
                new PIDConstants(1.0, 0, 0), // try without these?
                DriveConstants.maxWheelVelocity,
                Math.sqrt(2*Math.pow(DriveConstants.moduleToCenterDistance, 2)),
                new ReplanningConfig()
            ),
            () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
            this
        );

    }

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

    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
            states[i] = modules[i].getState();
        }

        odometry.update(gyro.getAngle(), positions);
        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("/Gyro", gyro.getAngle().getDegrees());
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("PDH/TotalCurrent", Robot.pdh.getTotalCurrent());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d p) {
        field.setRobotPose(p);
        odometry.resetPosition(gyro.getAngle(), positions, p);
    }
    
    public SwerveModulePosition[] getPositions() {
        return positions;
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(states);
    }

    public void drive(ChassisSpeeds speeds) {
        ChassisSpeeds currentSpeeds = kinematics.toChassisSpeeds(states);
        double currentRotation = gyro.getYawVelocity();
        ChassisSpeeds newSpeeds = new ChassisSpeeds(
            speeds.vxMetersPerSecond + 0.5 * (currentSpeeds.vxMetersPerSecond - speeds.vxMetersPerSecond), 
            speeds.vyMetersPerSecond + 0.5 * (currentSpeeds.vyMetersPerSecond - speeds.vyMetersPerSecond), 
            speeds.omegaRadiansPerSecond + 0.5 * (currentRotation - speeds.omegaRadiansPerSecond));
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(newSpeeds);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(desiredStates[i]);
        }
    }

    public void setX() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(xStates[i]);
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
