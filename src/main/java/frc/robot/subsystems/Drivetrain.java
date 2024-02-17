// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.Optional;
import java.util.concurrent.CancellationException;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI.Port;

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
  AHRS m_ahrs = new AHRS(Port.kMXP);

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

    m_leftFrontPID.setP(0.0001);
    m_leftFrontPID.setI(0.0);
    m_leftFrontPID.setD(0.0);
    m_leftFrontPID.setFF(0.01);

    m_rightFrontPID.setP(0.0001);
    m_rightFrontPID.setI(0.0);
    m_rightFrontPID.setD(0.0);
    m_rightFrontPID.setFF(0.01);

    m_rightFrontEncoder.setPositionConversionFactor(Units.inchesToMeters(6) * Math.PI);
    m_leftFrontEncoder.setPositionConversionFactor(Units.inchesToMeters(6) * Math.PI);

    m_rightFrontEncoder.setVelocityConversionFactor(Units.inchesToMeters(6) * Math.PI / 60);
    m_leftFrontEncoder.setVelocityConversionFactor(Units.inchesToMeters(6) * Math.PI / 60);

    m_differentialOdometry = new DifferentialDriveOdometry(m_ahrs.getRotation2d(), m_leftFrontEncoder.getPosition(), m_rightFrontEncoder.getPosition());

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
  }

  public void periodic() {
    m_differentialOdometry.update(m_ahrs.getRotation2d(), m_leftFrontEncoder.getPosition(), m_rightFrontEncoder.getPosition());
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
    return m_differentialOdometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_differentialOdometry.resetPosition(m_ahrs.getRotation2d(), m_leftFrontEncoder.getPosition(), m_rightFrontEncoder.getPosition(), pose);
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
