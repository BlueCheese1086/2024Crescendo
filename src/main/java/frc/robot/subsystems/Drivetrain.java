// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Drivetrain subsystem controlled with arcade-style inputs.
 */
public class Drivetrain extends SubsystemBase {
  CANSparkMax m_rightFront;
  CANSparkMax m_rightBack; 
  CANSparkMax m_leftFront;
  CANSparkMax m_leftBack;

  /**
   * Creates a new Drivetrain subsystem.
   */
  public Drivetrain(){
    // The CANSparkMax motors are being initalized.
    m_rightFront = new CANSparkMax(RightFrontMotor, MotorType.kBrushless);
    m_rightBack = new CANSparkMax(RightBackMotor, MotorType.kBrushless);
    m_leftFront = new CANSparkMax(LeftFrontMotor, MotorType.kBrushless);
    m_leftBack = new CANSparkMax(LeftBackMotor, MotorType.kBrushless);

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
  }

  /**
   * @param speed X-axis speed.
   * @param rotate Z-axis rotate.
   */
  public void arcadeDrive(double speed, double rotate) {
    m_leftFront.set((speed - rotate) * SpeedMultiplier);
    m_rightFront.set((speed + rotate) * SpeedMultiplier);
  }
}
