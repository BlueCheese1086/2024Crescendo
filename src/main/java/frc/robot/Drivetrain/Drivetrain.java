package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
    // Swerve modules for the robot -- NOT BUILT
    SwerveModule flModule = new SwerveModule("Front Left",  DriveConstants.frontLeftDriveID,  DriveConstants.frontLeftTurnID,  DriveConstants.frontLeftCancoderID,  DriveConstants.frontLeftOffset);
    SwerveModule blModule = new SwerveModule("Back Left",   DriveConstants.backLeftDriveID,   DriveConstants.backLeftTurnID,   DriveConstants.backLeftCancoderID,   DriveConstants.backLeftOffset);
    SwerveModule frModule = new SwerveModule("Front Right", DriveConstants.frontRightDriveID, DriveConstants.frontRightTurnID, DriveConstants.frontRightCancoderID, DriveConstants.frontRightOffset);
    SwerveModule brModule = new SwerveModule("Back Right",  DriveConstants.backRightDriveID,  DriveConstants.backRightTurnID,  DriveConstants.backRightCancoderID,  DriveConstants.backRightOffset);

    // Sensors
    // Pigeon2 gyro = new Pigeon2(DriveConstants.gyroID);

    // Module Management
    private SwerveModuleState[] states = new SwerveModuleState[4];

    // Kinematics
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d( DriveConstants.kModuleToCenter,  DriveConstants.kModuleToCenter),
        new Translation2d( DriveConstants.kModuleToCenter, -DriveConstants.kModuleToCenter),
        new Translation2d(-DriveConstants.kModuleToCenter,  DriveConstants.kModuleToCenter),
        new Translation2d(-DriveConstants.kModuleToCenter, -DriveConstants.kModuleToCenter)
    );

    /**
     * Constructor. This method is called when an instance of the class is created. This should generally be used to set up
     * instance variables and perform any configuration or necessary set up on hardware.
     */
    public Drivetrain() {}

    @Override
    public void periodic() {}

    /**
     * Drives the robot in different directions.
     * 
     * @param speeds The speeds that each motor should go at.
     */
    public void swerveDrive(ChassisSpeeds speeds) {
        // Converting the ChassisSpeeds to SwerveModuleStates
        states = kinematics.toSwerveModuleStates(speeds);

        // Setting the states of each module
        flModule.setState(states[0]);
        blModule.setState(states[1]);
        frModule.setState(states[2]);
        brModule.setState(states[3]);

        // Updating the Gyro if the robot is being simulated
        // if (Robot.isSimulation()) {
        //     gyro.setYaw(gyro.getYaw().getValueAsDouble() + 180 + (System.currentTimeMillis() - start) / 1000.0 * Units.radiansToDegrees(speeds.omegaRadiansPerSecond));
        // }
    }

    /**
     * Returns the current angle of the robot
     * 
     * @return The angle of the robot as a Rotation2D
     */
    public Rotation2d getAngle() {
        return new Rotation2d();//new Rotation2d(gyro.getAngle());
    }

    /**
     * Returns the angluar velocity along the z axis of the robot.
     * 
     * @return The rate that the robot is turning. (rad/sec)
     */
    public double getAngularVel() {
        return 0;//Math.toRadians(gyro.getRate());
    }
}