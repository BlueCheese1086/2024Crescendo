// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.DrivetrainLimits;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
    DifferentialDrive m_drivetrain;

  public Drivetrain(){
    // The CANSparkMax motors are being initalized.
    CANSparkMax rightFront = new CANSparkMax(1, MotorType.kBrushed);
    CANSparkMax rightBack = new CANSparkMax(2, MotorType.kBrushed);
    CANSparkMax leftFront = new CANSparkMax(3, MotorType.kBrushed);
    CANSparkMax leftBack = new CANSparkMax(4, MotorType.kBrushed);

    leftFront.setSmartCurrentLimit(DrivetrainLimits);
    leftBack.setSmartCurrentLimit(DrivetrainLimits);
    rightFront.setSmartCurrentLimit(DrivetrainLimits);
    rightBack.setSmartCurrentLimit(DrivetrainLimits);
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
}
