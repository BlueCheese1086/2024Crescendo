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

public class Launcher extends SubsystemBase {
  CANSparkMax m_lower = new CANSparkMax(LowerMotor, MotorType.kBrushless);
  CANSparkMax m_upper = new CANSparkMax(UpperMotor, MotorType.kBrushless);

  SparkPIDController lowerPID = m_lower.getPIDController();
  SparkPIDController upperPID = m_upper.getPIDController();

  public Launcher() {
    // The CANSparkMax motors are being initalized.
    m_lower.restoreFactoryDefaults();
    m_upper.restoreFactoryDefaults();

    lowerPID.setP(0.00001);
    lowerPID.setI(0);
    lowerPID.setD(0);
    lowerPID.setFF(0.01);

    upperPID.setP(0.00001);
    upperPID.setI(0);
    upperPID.setD(0);
    upperPID.setFF(0.01);

    m_lower.setIdleMode(IdleMode.kCoast);
    m_upper.setIdleMode(IdleMode.kCoast);

    m_lower.setInverted(true);
    m_upper.setInverted(true);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor). lil'python was here
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Sets the speed of the upper (flywheel) motor.
   * @param speed The desired speed in RPM
   */
  public void setUpper(double speed) {
    System.out.println(speed);
    upperPID.setReference(speed, ControlType.kVelocity);
  }

  /**
   * Sets the speed of the lower (feed) motor.
   * @param speed The desired speed in RPM
   */
  public void setLower(double speed) {
    System.out.println(speed);
    lowerPID.setReference(speed, ControlType.kVelocity);
  }

  public void stopUpper() {
    m_upper.stopMotor();
  }

  public void stopLower() {
    m_lower.stopMotor();
  }
}