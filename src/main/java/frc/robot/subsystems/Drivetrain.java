// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.concurrent.CancellationException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * Drivetrain subsystem controlled with arcade-style inputs.
 */
public class Drivetrain extends SubsystemBase {
  CANSparkMax m_rightFront = new CANSparkMax(RightFrontMotor, MotorType.kBrushless);
  CANSparkMax m_rightBack = new CANSparkMax(RightBackMotor, MotorType.kBrushless);
  CANSparkMax m_leftFront = new CANSparkMax(LeftFrontMotor, MotorType.kBrushless);
  CANSparkMax m_leftBack = new CANSparkMax(LeftBackMotor, MotorType.kBrushless);

  SparkPIDController m_rightFrontPID = m_rightFront.getPIDController();
  SparkPIDController m_leftFrontPID = m_leftFront.getPIDController();

  RelativeEncoder m_rightFrontEncoder = m_rightFront.getEncoder();
  RelativeEncoder m_leftFrontEncoder = m_leftFront.getEncoder();

  Pose2d m_pose = new Pose2d();

  DifferentialDriveKinematics m_differentialKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
  DifferentialDriveOdometry m_differentialOdometry;

  /**
   * Creates a new Drivetrain subsystem.
   */
  public Drivetrain() {
    m_rightFront.restoreFactoryDefaults();
    m_rightBack.restoreFactoryDefaults();
    m_leftFront.restoreFactoryDefaults();
    m_leftBack.restoreFactoryDefaults();

    m_leftFront.setSmartCurrentLimit(DrivetrainLimits);
    m_leftBack.setSmartCurrentLimit(DrivetrainLimits);
    m_rightFront.setSmartCurrentLimit(DrivetrainLimits);
    m_rightBack.setSmartCurrentLimit(DrivetrainLimits);

    m_leftFront.setIdleMode(IdleMode.kBrake);
    m_leftBack.setIdleMode(IdleMode.kBrake);
    m_rightFront.setIdleMode(IdleMode.kBrake);
    m_rightBack.setIdleMode(IdleMode.kBrake);

    m_rightFront.setInverted(true);
    m_leftFront.setInverted(false);

    m_rightBack.follow(m_rightFront);
    m_leftBack.follow(m_leftFront);

    m_rightFrontEncoder.setPositionConversionFactor(Units.inchesToMeters(6) * 1 * Math.PI / 60);
    m_leftFrontEncoder.setPositionConversionFactor(Units.inchesToMeters(6) * 1 * Math.PI / 60);

    m_differentialOdometry = new DifferentialDriveOdometry(null, m_leftFrontEncoder.getPosition(), m_rightFrontEncoder.getPosition());

    AutoBuilder.configureRamsete(
      this::getPose, this::resetPose, this::getSpeeds, this::setSpeeds, new ReplanningConfig(), () -> false, this
    );
  }

  /**
   * @param speed X-axis speed.
   * @param rotate Z-axis rotate.
   */
  public void arcadeDrive(double speed, double rotate) {
    m_leftFrontPID.setReference((speed - rotate) * DrivetrainSpeed, ControlType.kVelocity);
    m_rightFrontPID.setReference((speed + rotate) * DrivetrainSpeed, ControlType.kVelocity);
  }

  public Pose2d getPose() {
    return new Pose2d(

    );
  }

  public void resetPose(Pose2d pose) {
    m_differentialOdometry.resetPosition(null, m_leftFrontEncoder.getPosition(), m_rightFrontEncoder.getPosition(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return m_differentialKinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(
        m_leftFrontEncoder.getVelocity(),
        m_rightFrontEncoder.getVelocity()
      )
    );
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_differentialKinematics.toWheelSpeeds(speeds);
    m_leftFrontPID.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity);
    m_rightFrontPID.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
  }
}
