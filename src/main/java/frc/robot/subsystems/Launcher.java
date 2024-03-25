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

    upperEncoder.setVelocityConversionFactor(4 * Math.PI / 60);
    lowerEncoder.setVelocityConversionFactor(4 * Math.PI / 60);
  }
 
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Speed", upperEncoder.getVelocity());
    SmartDashboard.putNumber("Feed Wheel Speed", lowerEncoder.getVelocity());
  }

  /**
   * Runs the flywheel
   */
  public void flywheel() {
    SmartDashboard.putNumber("Flywheel Speed Setpoint", FlywheelSpeed);
    upperPID.setReference(FlywheelSpeed, ControlType.kVelocity);
  }

  /**
   * Runs the feed wheel
   */
  public void feed() {
    SmartDashboard.putNumber("Feed Wheel Speed Setpoint", FeedSpeed);
    lowerPID.setReference(FeedSpeed, ControlType.kVelocity);
  }

  /**
   * Runs the wheels in reverse to intake a note
   */
  public void intake() {
    SmartDashboard.putNumber("Feed Wheel Speed Setpoint", IntakeSpeed);
    SmartDashboard.putNumber("Flywheel Speed Setpoint", IntakeSpeed);

    lowerPID.setReference(IntakeSpeed, ControlType.kVelocity);
    upperPID.setReference(IntakeSpeed, ControlType.kVelocity);
  }
}