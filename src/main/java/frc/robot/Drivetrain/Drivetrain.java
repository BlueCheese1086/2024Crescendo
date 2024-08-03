// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    // Motors
    private CANSparkMax flMotor;
    private CANSparkMax frMotor;
    private CANSparkMax blMotor;
    private CANSparkMax brMotor;

    // Encoders
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    // PID Controllers
    private SparkPIDController leftPID;
    private SparkPIDController rightPID;

    // Gyro
    private PigeonIMU gyro;

    // Kinematics
    private DifferentialDriveKinematics kinematics;

    // Odometry
    private Pose2d initPose = new Pose2d();
    private DifferentialDrivePoseEstimator poseEstimator;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) instance = new Drivetrain();

        return instance;
    }

    /** Creates a new ExampleSubsystem. */
    public Drivetrain() {
        // Getting motors
        flMotor = new CANSparkMax(DriveConstants.frontLeftID, MotorType.kBrushless);
        frMotor = new CANSparkMax(DriveConstants.frontRightID, MotorType.kBrushless);
        blMotor = new CANSparkMax(DriveConstants.backLeftID, MotorType.kBrushless);
        brMotor = new CANSparkMax(DriveConstants.backRightID, MotorType.kBrushless);

        // Resetting motors
        flMotor.restoreFactoryDefaults();
        frMotor.restoreFactoryDefaults();
        blMotor.restoreFactoryDefaults();
        brMotor.restoreFactoryDefaults();

        // Creating followers
        blMotor.follow(flMotor);
        brMotor.follow(frMotor);

        // Inverting motors
        blMotor.setInverted(true);
        brMotor.setInverted(false);
        flMotor.setInverted(true);
        frMotor.setInverted(false);

        // Setting idle modes
        flMotor.setIdleMode(IdleMode.kBrake);
        blMotor.setIdleMode(IdleMode.kBrake);
        frMotor.setIdleMode(IdleMode.kBrake);
        brMotor.setIdleMode(IdleMode.kBrake);

        // Getting encoders
        leftEncoder = flMotor.getEncoder();
        rightEncoder = frMotor.getEncoder();

        // Getting gyro
        gyro = new PigeonIMU(10);
        gyro.setYaw(0);

        // Setting conversion factors
        leftEncoder.setPositionConversionFactor(DriveConstants.posConversionFactor);
        leftEncoder.setVelocityConversionFactor(DriveConstants.velConversionFactor);
        rightEncoder.setPositionConversionFactor(DriveConstants.posConversionFactor);
        rightEncoder.setVelocityConversionFactor(DriveConstants.velConversionFactor);

        // Getting PID controllers
        leftPID = flMotor.getPIDController();
        rightPID = frMotor.getPIDController();

        // Saving configs
        brMotor.burnFlash();
        blMotor.burnFlash();
        frMotor.burnFlash();
        flMotor.burnFlash();

        // Initializing the kinematics
        kinematics = new DifferentialDriveKinematics(DriveConstants.robotWidth);

        // Initializing the pose estimator
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, getAngle(), getLeftPosition(), getRightPosition(), initPose);

        // Initializing the autobuilder
        AutoBuilder.configureRamsete(
            this::getPose,
            this::setPose,
            this::getSpeeds,
            this::closedLoop,
            new ReplanningConfig(),
            () -> (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red),
            this
        );
    }

    @Override
    public void periodic() {


        SmartDashboard.putNumber("/Drivetrain/Left_Actual_MPS", 0);
        SmartDashboard.putNumber("/Drivetrain/Left_Expected_MPS", 0);
        SmartDashboard.putNumber("/Drivetrain/Right_Actual_MPS", 0);
        SmartDashboard.putNumber("/Drivetrain/Right_Expected_MPS", 0);
        SmartDashboard.putNumber("/Drivetrain/Angle", getAngle().getRadians());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d newPose) {
        poseEstimator.resetPosition(getAngle(), getPosition(), newPose);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity()));
    }

    public DifferentialDriveWheelPositions getPosition() {
        return new DifferentialDriveWheelPositions(getLeftPosition(), getRightPosition());
    }

    public double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    public double getLeftVelocity() {
        return leftEncoder.getVelocity();
    }

    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }
    
    public void arcadeDrive(double xSpeed, double zRotate) {
        xSpeed *= DriveConstants.maxDriveSpeed;
        zRotate *= DriveConstants.maxTurnSpeed;

        if (xSpeed < 0.1 && xSpeed > -0.1) xSpeed = 0;
        if (zRotate < 0.1 && zRotate > -0.1) zRotate = 0;

        if (xSpeed  != 0 && xSpeed  > 0) xSpeed  -= 0.1;
        if (xSpeed  != 0 && xSpeed  < 0) xSpeed  += 0.1;
        if (zRotate != 0 && zRotate > 0) zRotate -= 0.1;
        if (zRotate != 0 && zRotate < 0) zRotate += 0.1;

        flMotor.set(xSpeed - zRotate);
        frMotor.set(xSpeed + zRotate);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftSpeed *= DriveConstants.maxDriveSpeed;
        rightSpeed *= DriveConstants.maxTurnSpeed;

        if (leftSpeed < 0.1 && leftSpeed > -0.1) leftSpeed = 0;
        if (rightSpeed < 0.1 && rightSpeed > -0.1) rightSpeed = 0;

        if (leftSpeed  != 0 && leftSpeed  > 0) leftSpeed  -= 0.1;
        if (leftSpeed  != 0 && leftSpeed  < 0) leftSpeed  += 0.1;
        if (rightSpeed != 0 && rightSpeed > 0) rightSpeed -= 0.1;
        if (rightSpeed != 0 && rightSpeed < 0) rightSpeed += 0.1;

        flMotor.set(leftSpeed);
        frMotor.set(rightSpeed);
    }

    public void closedLoop(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds newSpeeds = kinematics.toWheelSpeeds(speeds);

        leftPID.setReference(newSpeeds.leftMetersPerSecond, ControlType.kVelocity);
        rightPID.setReference(newSpeeds.rightMetersPerSecond, ControlType.kVelocity);
    }
}