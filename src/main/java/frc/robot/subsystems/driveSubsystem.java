// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;

public class driveSubsystem extends SubsystemBase {
  CANSparkMax rightLeader = new CANSparkMax(Constants.DriveConstants.FRONT_RIGHT_ID, MotorType.kBrushless); 
  CANSparkMax rightFollower = new CANSparkMax(Constants.DriveConstants.BACK_RIGHT_ID, MotorType.kBrushless);
  CANSparkMax leftLeader = new CANSparkMax(Constants.DriveConstants.FRONT_LEFT_ID, MotorType.kBrushless);
  CANSparkMax leftFollower = new CANSparkMax(Constants.DriveConstants.BACK_LEFT_ID, MotorType.kBrushless);

  RelativeEncoder encoderFR = rightLeader.getEncoder();
  RelativeEncoder encoderFL = leftLeader.getEncoder();
  RelativeEncoder encoderBR = rightFollower.getEncoder();
  RelativeEncoder encoderBL = leftFollower.getEncoder();

  SparkPIDController FLPID = leftLeader.getPIDController();
  SparkPIDController FRPID = rightLeader.getPIDController();

  PIDController drivePID = new PIDController(0.1, 0, 0);

  ChassisSpeeds superChassisSpeeds = new ChassisSpeeds(0, 0, 0);

  CommandXboxController xbox = new CommandXboxController(0);
  XboxController joy = xbox.getHID();

  double superLeftSpeed;
  double superRightSpeed;

  AHRS gyro = new AHRS(SPI.Port.kMXP);
  Rotation2d rotation = new Rotation2d();

  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
  DifferentialDriveWheelSpeeds difSpeeds = new DifferentialDriveWheelSpeeds();

  public driveSubsystem() {

    rightLeader.restoreFactoryDefaults();
    leftLeader.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();

    rightLeader.setIdleMode(IdleMode.kBrake);
    leftLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);

    leftLeader.setSmartCurrentLimit(DriveConstants.DRIVETRAINLIMITS);
    leftFollower.setSmartCurrentLimit(DriveConstants.DRIVETRAINLIMITS);
    rightLeader.setSmartCurrentLimit(DriveConstants.DRIVETRAINLIMITS);
    rightFollower.setSmartCurrentLimit(DriveConstants.DRIVETRAINLIMITS);

    leftLeader.setInverted(false);
    rightLeader.setInverted(true);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader); 

    FLPID.setP(DriveConstants.DriveP);
    FLPID.setI(DriveConstants.DriveI);
    FLPID.setD(DriveConstants.DriveD);
    FLPID.setFF(DriveConstants.DriveFF);

    FRPID.setP(DriveConstants.DriveP);
    FRPID.setI(DriveConstants.DriveI);
    FRPID.setD(DriveConstants.DriveD);
    FRPID.setFF(DriveConstants.DriveFF);

    encoderFL.setPosition(0);
    encoderFR.setPosition(0);
    gyro.reset();
    encoderFL.setPositionConversionFactor(DriveConstants.driveRatio);
    encoderFR.setPositionConversionFactor(DriveConstants.driveRatio);
    encoderFL.setVelocityConversionFactor(DriveConstants.driveRatio / 60);
    encoderFR.setVelocityConversionFactor(DriveConstants.driveRatio / 60);

    AutoBuilder.configureRamsete(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      //this::getSpeeds, // Current ChassisSpeeds supplier
      () ->
      kinematics.toChassisSpeeds(
          new DifferentialDriveWheelSpeeds(
              getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),
      //this::driveChassis, // Method that will drive the robot given ChassisSpeeds
      (speeds) -> {
        SmartDashboard.putNumber("Pathplanner vx", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Pathplanner vy", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Pathplanner Omega Radians", speeds.omegaRadiansPerSecond);
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        driveChassis(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
      },
      new ReplanningConfig(), // Default path replanning config. See the API for the options here
      () -> {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), encoderFL.getPosition(), encoderFR.getPosition());
  }

  /*public void set(double rotateSpeed, double driveSpeed) {
    rightLeader.set((driveSpeed + rotateSpeed));
    leftLeader.set((driveSpeed - rotateSpeed));
    
    SmartDashboard.putNumber("Rotate speed:", rotateSpeed);
    SmartDashboard.putNumber("Drive speed:", driveSpeed);
    SmartDashboard.putNumber("Joystick x", joy.getLeftX());
    SmartDashboard.putNumber("Joystick y", joy.getLeftY());
    SmartDashboard.putNumber("FL Rotations", encoderFL.getPosition());
    SmartDashboard.putNumber("FR Rotations", encoderFR.getPosition());
    SmartDashboard.putNumber("Counts per rev", encoderFL.getCountsPerRevolution());
  }*/

  public RelativeEncoder getEncoderFL(){
    return encoderFL;
  }

  public RelativeEncoder getEncoderFR(){
    return encoderFR;
  }

  public void driveAlign(double yaw){
    MathUtil.applyDeadband(yaw, 2);
    rightLeader.set(drivePID.calculate(yaw, 0) * DriveConstants.MAX_ALIGN_SPEED);
    leftLeader.set(-drivePID.calculate(yaw, 0) * DriveConstants.MAX_ALIGN_SPEED);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose){
    odometry.resetPosition(gyro.getRotation2d(), encoderFL.getPosition(), encoderFR.getPosition(), pose);
  }

  public void driveChassis(double leftSpeed, double rightSpeed){
    //ChassisSpeeds speeds
    //difSpeeds = kinematics.toWheelSpeeds(speeds);
    //FRPID.setReference(difSpeeds.rightMetersPerSecond, ControlType.kVelocity);
    //FLPID.setReference(difSpeeds.leftMetersPerSecond, ControlType.kVelocity);
    //SmartDashboard.putNumber("Chassis vx", speeds.vxMetersPerSecond);
    //SmartDashboard.putNumber("Chassis vy", speeds.vyMetersPerSecond);
    superLeftSpeed = leftSpeed;
    superRightSpeed = rightSpeed;
    FLPID.setReference(leftSpeed, ControlType.kVelocity);
    FRPID.setReference(rightSpeed, ControlType.kVelocity);
    SmartDashboard.putNumber("Dif Left", leftSpeed);
    SmartDashboard.putNumber("Dif Right", rightSpeed);
    SmartDashboard.putNumber("FL Meters", encoderFL.getPosition());
    SmartDashboard.putNumber("FR Meters", encoderFR.getPosition());
  }

  public ChassisSpeeds getSpeeds(){
    return superChassisSpeeds;
  }

  public double getLeftVelocityMetersPerSec(){
    return superLeftSpeed;
  }

  public double getRightVelocityMetersPerSec(){
    return superRightSpeed;
  }
}
