package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
    // The motors for the drivetrain subsystem
    CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.FrontLeftID, MotorType.kBrushless);
    CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.BackLeftID, MotorType.kBrushless);
    CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.FrontRightID, MotorType.kBrushless);
    CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.BackRightID, MotorType.kBrushless);

    // Swerve modules for the robot -- NOT BUILT
    SwerveModule flModule = new SwerveModule("Front Left", SwerveConstants.frontLeftDriveID, SwerveConstants.frontLeftTurnID, SwerveConstants.frontLeftCancoderID, SwerveConstants.frontLeftOffset);
    SwerveModule blModule = new SwerveModule("Back Left", SwerveConstants.backLeftDriveID, SwerveConstants.backLeftTurnID, SwerveConstants.backLeftCancoderID, SwerveConstants.backLeftOffset);
    SwerveModule frModule = new SwerveModule("Front Right", SwerveConstants.frontRightDriveID, SwerveConstants.frontRightTurnID, SwerveConstants.frontRightCancoderID, SwerveConstants.frontRightOffset);
    SwerveModule brModule = new SwerveModule("Back Right", SwerveConstants.backRightDriveID, SwerveConstants.backRightTurnID, SwerveConstants.backRightCancoderID, SwerveConstants.backRightOffset);

    // Sensors
    Pigeon2 gyro = new Pigeon2(DriveConstants.gyroID);

    // Module Management
    private final SwerveModule[] modules = {flModule, frModule, blModule, brModule};
    private SwerveModuleState[] states = new SwerveModuleState[4];
    private final SwerveModulePosition[] positions = new SwerveModulePosition[4];

    // Kinematics
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(SwerveConstants.kModuleToCenter, SwerveConstants.kModuleToCenter), new Translation2d(SwerveConstants.kModuleToCenter, -SwerveConstants.kModuleToCenter), new Translation2d(-SwerveConstants.kModuleToCenter, SwerveConstants.kModuleToCenter), new Translation2d(-SwerveConstants.kModuleToCenter, -SwerveConstants.kModuleToCenter));
    private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

    // Odometry
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), positions);
    private final Field2d field = new Field2d();

    // Timer
    private final double start;

    /**
     * Constructor. This method is called when an instance of the class is created. This should generally be used to set up
     * instance variables and perform any configuration or necessary set up on hardware.
     */
    public Drivetrain() {
        // Setting the start time
        start = System.currentTimeMillis();

        // Setting the default settings for each motor
        frontLeftMotor.restoreFactoryDefaults();
        frontLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
        frontLeftMotor.setInverted(true);
        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        frontLeftMotor.burnFlash();

        backLeftMotor.restoreFactoryDefaults();
        backLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
        backLeftMotor.setInverted(true);
        backLeftMotor.setIdleMode(IdleMode.kBrake);
        backLeftMotor.follow(frontLeftMotor);
        backLeftMotor.burnFlash();

        frontRightMotor.restoreFactoryDefaults();
        frontRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
        frontRightMotor.setInverted(false);
        frontRightMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.burnFlash();

        backRightMotor.restoreFactoryDefaults();
        backRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
        backRightMotor.setInverted(false);
        backRightMotor.setIdleMode(IdleMode.kBrake);
        backRightMotor.follow(frontRightMotor);
        backRightMotor.burnFlash();
    }

    /**
     * This function runs every tick.
     * I am using it to update the odometry of the motors,
     * 
     */
    @Override
    public void periodic() {

    }

    /**
     * Yet to be implemented -- This is theoretical code for when Kitbot gets the swerve upgrade.
     * Drives the robot in different directions determined by the parameters.
     * 
     * @param xTraverse
     * @param zTraverse
     * @param zRotate
     */
    public void swerveDrive(ChassisSpeeds speeds) {
        // Updating the local speeds variable
        this.speeds = speeds;

        // Converting the ChassisSpeeds to SwerveModuleStates
        states = kinematics.toSwerveModuleStates(speeds);

        // Setting the states of each module
        flModule.setState(states[0]);
        blModule.setState(states[1]);
        frModule.setState(states[2]);
        brModule.setState(states[3]);

        // Updating the Gyro if the robot is being simulated
        if (Robot.isSimulation()) {
            gyro.setYaw(gyro.getYaw().getValueAsDouble() + 180 + (System.currentTimeMillis() - start) / 1000.0 * Units.radiansToDegrees(speeds.omegaRadiansPerSecond));
        }
    }

    /**
     * Drives the robot at the desired forward speed combined with the rotational speed.
     * 
     * @param xSpeed The desired forward speed of the robot.
     * @param zRotation The desired rotational speed of the robot.
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        // Applies a deadband to the inputs.
        MathUtil.applyDeadband(xSpeed, DriveConstants.deadband);
        MathUtil.applyDeadband(zRotation, DriveConstants.deadband);

        // Squares the inputs (while preserving the sign) to increase fine control while permitting full power.
        xSpeed = Math.abs(xSpeed) * xSpeed;
        zRotation = Math.abs(zRotation) * zRotation;

        // Creates the saturated speeds of the motors
        double leftSpeed = xSpeed - zRotation;
        double rightSpeed = xSpeed + zRotation;

        // Finds the maximum possible value of throttle + turn along the vector that the joystick is pointing, and then desaturates the wheel speeds.
        double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
        double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotation));
        if (greaterInput == 0.0) {
            leftSpeed = 0;
            rightSpeed = 0;
        } else {
            double saturatedInput = (greaterInput + lesserInput) / greaterInput;
            leftSpeed /= saturatedInput;
            rightSpeed /= saturatedInput;
        }

        // Sets the speed of the motors.
        frontLeftMotor.set(leftSpeed * DriveConstants.maxSpeed);
        frontRightMotor.set(rightSpeed * DriveConstants.maxSpeed);
    }

    /**
     * Returns the current angle of the robot
     * 
     * @return The angle of the robot as a Rotation2D
     */
    public Rotation2d getAngle() {
        return new Rotation2d();
    }
}