// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.DrivetrainLimits;
import static frc.robot.Constants.DrivetrainConstants.LeftBackMotor;
import static frc.robot.Constants.DrivetrainConstants.LeftFrontMotor;
import static frc.robot.Constants.DrivetrainConstants.RightBackMotor;
import static frc.robot.Constants.DrivetrainConstants.RightFrontMotor;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Drivetrain subsystem controlled with arcade-style inputs.
 */
public class Drivetrain extends SubsystemBase {
  CANSparkMax rightFront = new CANSparkMax(RightFrontMotor, MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(RightBackMotor, MotorType.kBrushless);
  CANSparkMax leftFront = new CANSparkMax(LeftFrontMotor, MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(LeftBackMotor, MotorType.kBrushless);

  SparkPIDController rightFrontPID = rightFront.getPIDController();
  SparkPIDController leftFrontPID = leftFront.getPIDController();

  RelativeEncoder rightFrontEncoder = rightFront.getEncoder();
  RelativeEncoder leftFrontEncoder = leftFront.getEncoder();

  Pose2d pose = new Pose2d();
  Pigeon2 pigeon = new Pigeon2(0);

  DifferentialDriveKinematics differentialKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
  DifferentialDriveOdometry differentialOdometry;

  Field2d field = new Field2d();

  /**
   * Creates a new Drivetrain subsystem.
   */
  public Drivetrain() {
    rightFront.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();
    leftFront.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();

    leftFront.setSmartCurrentLimit(DrivetrainLimits);
    leftBack.setSmartCurrentLimit(DrivetrainLimits);
    rightFront.setSmartCurrentLimit(DrivetrainLimits);
    rightBack.setSmartCurrentLimit(DrivetrainLimits);

    leftFront.setIdleMode(IdleMode.kBrake);
    leftBack.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);
    rightBack.setIdleMode(IdleMode.kBrake);

    rightFront.setInverted(true);
    leftFront.setInverted(false);

    rightBack.follow(rightFront);
    leftBack.follow(leftFront);

    leftFrontPID.setP(0.25);
    leftFrontPID.setI(0.0);
    leftFrontPID.setD(0.0);
    leftFrontPID.setFF(0.0);

    rightFrontPID.setP(0.25);
    rightFrontPID.setI(0.0);
    rightFrontPID.setD(0.0);
    rightFrontPID.setFF(0.0);

    rightFrontEncoder.setPositionConversionFactor(Units.inchesToMeters(6) * Math.PI / 10.75);
    leftFrontEncoder.setPositionConversionFactor(Units.inchesToMeters(6) * Math.PI / 10.75);

    rightFrontEncoder.setVelocityConversionFactor(Units.inchesToMeters(6) * Math.PI / 10.75 / 60);
    leftFrontEncoder.setVelocityConversionFactor(Units.inchesToMeters(6) * Math.PI / 10.75 / 60);

    pigeon.setYaw(0);

    differentialOdometry = new DifferentialDriveOdometry(pigeon.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());

    AutoBuilder.configureRamsete(
      this::getPose,
      this::resetPose,
      this::getSpeeds,
      this::setSpeeds,
      new ReplanningConfig(), 
      () -> {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );

    SmartDashboard.putData("Field", field);
  }

  public void periodic() {
    differentialOdometry.update(pigeon.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
    SmartDashboard.putNumber("Left Speed", leftFrontEncoder.getVelocity());
    SmartDashboard.putNumber("Right Speed", rightFrontEncoder.getVelocity());
    SmartDashboard.putNumber("Angle", pigeon.getRotation2d().getDegrees());
    field.setRobotPose(pose);
  }

  public Pose2d getPose() {
    return differentialOdometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    differentialOdometry.resetPosition(pigeon.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return differentialKinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(
        leftFrontEncoder.getVelocity(),
        rightFrontEncoder.getVelocity()
      )
    );
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = differentialKinematics.toWheelSpeeds(speeds);
    SmartDashboard.putNumber("Left Speed Setpoint", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Speed Setpoint", wheelSpeeds.rightMetersPerSecond);

    leftFrontPID.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity);
    rightFrontPID.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
  }
}