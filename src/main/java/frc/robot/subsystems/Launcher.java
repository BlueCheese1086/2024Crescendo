// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  CANSparkMax m_lower;
  CANSparkMax m_upper; 

  public Launcher(){
    // The CANSparkMax motors are being initalized.
    m_lower = new CANSparkMax(LowerMotor, MotorType.kBrushless);
    m_upper = new CANSparkMax(UpperMotor, MotorType.kBrushless);

    m_lower.restoreFactoryDefaults();
    m_upper.restoreFactoryDefaults();

    m_lower.setSmartCurrentLimit(LauncherLimits);
    m_upper.setSmartCurrentLimit(LauncherLimits);

    m_lower.setIdleMode(IdleMode.kCoast);
    m_upper.setIdleMode(IdleMode.kCoast);

    m_lower.setInverted(false);
    m_upper.setInverted(false);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setUpper(double speed) {
    m_upper.set(speed);
  }

  public void setLower(double speed) {
    m_lower.set(speed);
  }
}
