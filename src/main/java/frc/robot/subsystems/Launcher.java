// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Launcher subsystem uses PID and FeedForward to control upper (flywheel) motor and launch (feed) 
 * motor.
 */
public class Launcher extends SubsystemBase {
  CANSparkMax m_lower = new CANSparkMax(LowerMotor, MotorType.kBrushless);
  CANSparkMax m_upper = new CANSparkMax(UpperMotor, MotorType.kBrushless);

  SparkPIDController m_lowerPID = m_lower.getPIDController();
  SparkPIDController m_upperPID = m_upper.getPIDController();

  /**
   * Creates a new launcher subsystem.
   */
  public Launcher() {
    // The CANSparkMax motors are being initalized.
    m_lower.restoreFactoryDefaults();
    m_upper.restoreFactoryDefaults();

    m_lower.setIdleMode(IdleMode.kCoast);
    m_upper.setIdleMode(IdleMode.kCoast);

    m_lower.setInverted(true);
    m_upper.setInverted(true);

    m_lowerPID.setP(0.00001);
    m_lowerPID.setI(0);
    m_lowerPID.setD(0);
    m_lowerPID.setFF(0.01);

    m_upperPID.setP(0.00001);
    m_upperPID.setI(0);
    m_upperPID.setD(0);
    m_upperPID.setFF(0.01);
  }

  /**
   * Sets the speed of the upper (flywheel) motor.
   * @param speed The desired speed in RPM
   */
  public void setUpper(double speed) {
    System.out.println(speed);
    m_upperPID.setReference(speed, ControlType.kVelocity);
  }

  /**
   * Sets the speed of the lower (feed) motor.
   * @param speed The desired speed in RPM
   */
  public void setLower(double speed) {
    System.out.println(speed);
    m_lowerPID.setReference(speed, ControlType.kVelocity);
  }

  /**
   * Stops the upper (flywheel) motor, which stops the control systems.
   */
  public void stopUpper() {
    m_upper.stopMotor();
  }

  /**
   * Stops the lower (feed) motor, which stops the control systems.
   */
  public void stopLower() {
    m_lower.stopMotor();
  }
}