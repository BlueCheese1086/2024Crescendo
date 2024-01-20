// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.DrivetrainLimits;
import static frc.robot.Constants.DrivetrainConstants.LeftBackMotor;
import static frc.robot.Constants.DrivetrainConstants.LeftFrontMotor;
import static frc.robot.Constants.DrivetrainConstants.RightBackMotor;
import static frc.robot.Constants.DrivetrainConstants.RightFrontMotor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final DifferentialDrive m_diffDrive;

  public Drivetrain(){
    // The CANSparkMax motors are being initalized.
    CANSparkMax rightFront = new CANSparkMax(RightFrontMotor, MotorType.kBrushless);
    CANSparkMax rightBack = new CANSparkMax(RightBackMotor, MotorType.kBrushless);
    CANSparkMax leftFront = new CANSparkMax(LeftFrontMotor, MotorType.kBrushless);
    CANSparkMax leftBack = new CANSparkMax(LeftBackMotor, MotorType.kBrushless);

    leftFront.setSmartCurrentLimit(DrivetrainLimits);
    leftBack.setSmartCurrentLimit(DrivetrainLimits);
    rightFront.setSmartCurrentLimit(DrivetrainLimits);
    rightBack.setSmartCurrentLimit(DrivetrainLimits);

    rightFront.setInverted(false);
    leftFront.setInverted(true);

    rightBack.follow(rightFront);
    leftBack.follow(leftFront);

    m_diffDrive = new DifferentialDrive(leftFront, rightFront);
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

  public void arcadeDrive(double speed, double rotate) {
    m_diffDrive.arcadeDrive(speed, rotate);
  }
}
