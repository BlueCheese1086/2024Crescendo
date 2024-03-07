// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Launcher subsystem uses PID and FeedForward to control upper (flywheel) motor and launch (feed) 
 * motor.
 */
public class Launcher extends SubsystemBase {
  CANSparkMax lower = new CANSparkMax(LowerMotor, MotorType.kBrushless);
  CANSparkMax upper = new CANSparkMax(UpperMotor, MotorType.kBrushless);

  SparkPIDController lowerPID = lower.getPIDController();
  SparkPIDController upperPID = upper.getPIDController();

  RelativeEncoder lowerEncoder = lower.getEncoder();
  RelativeEncoder upperEncoder = upper.getEncoder();

  /**
   * Creates a new launcher subsystem.
   */
  public Launcher() {
    // The CANSparkMax motors are being initalized.
    lower.restoreFactoryDefaults();
    upper.restoreFactoryDefaults();

    lower.setIdleMode(IdleMode.kCoast);
    upper.setIdleMode(IdleMode.kCoast);

    lower.setInverted(true);
    upper.setInverted(true);

    lowerPID.setP(0.00001);
    lowerPID.setI(0);
    lowerPID.setD(0);
    lowerPID.setFF(0.01);

    upperPID.setP(0.00001);
    upperPID.setI(0);
    upperPID.setD(0);
    upperPID.setFF(0.01);

    // INSERT WHEEL DIAMETER
    upperEncoder.setVelocityConversionFactor(1 * Math.PI / 60);
    lowerEncoder.setVelocityConversionFactor(1 * Math.PI / 60);
  }
 
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Speed", upperEncoder.getVelocity());
    SmartDashboard.putNumber("Feed Speed", lowerEncoder.getVelocity());
  }

  /**
   * Sets the speed of the upper (flywheel) motor.
   * @param speed The desired speed in RPM
   */
  public void setUpper(double speed) {
    SmartDashboard.putNumber("Flywheel Speed Setpoint", speed);
    upperPID.setReference(speed, ControlType.kVelocity);
  }

  /**
   * Sets the speed of the lower (feed) motor.
   * @param speed The desired speed in RPM
   */
  public void setLower(double speed) {
    SmartDashboard.putNumber("Feed Speed Setpoint", speed);
    lowerPID.setReference(speed, ControlType.kVelocity);
  }

  /**
   * Stops the upper (flywheel) motor, which stops the control systems.
   */
  public void stopUpper() {
    upper.stopMotor();
  }

  /**
   * Stops the lower (feed) motor, which stops the control systems.
   */
  public void stopLower() {
    lower.stopMotor();
  }
}