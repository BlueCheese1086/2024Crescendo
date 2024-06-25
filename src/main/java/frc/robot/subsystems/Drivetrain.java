// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ram;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    CANSparkMax leftMotor;
    CANSparkMax leftBackMotor;
    CANSparkMax rightMotor;
    CANSparkMax rightBackMotor;

    SparkPIDController leftPID;
    SparkPIDController rightPID;

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {

    leftMotor = new CANSparkMax(DriveConstants.frontLeftID, MotorType.kBrushless);
    leftBackMotor = new CANSparkMax(DriveConstants.backLeftID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(DriveConstants.frontRightID, MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(DriveConstants.backRightID, MotorType.kBrushless);

    rightBackMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    leftMotor.restoreFactoryDefaults();

    leftBackMotor.follow(leftMotor);
    rightBackMotor.follow(rightMotor);

    leftBackMotor.setInverted(true);
    rightBackMotor.setInverted(false);
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    leftMotor.setIdleMode(IdleMode.kBrake);
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBrake);

    rightBackMotor.burnFlash();
    leftBackMotor.burnFlash();
    rightMotor.burnFlash();
    leftMotor.burnFlash();

  }
    
  public void drive(double xSpeed, double zRotate) {
    xSpeed *= DriveConstants.maxDriveSpeed;
    zRotate *= DriveConstants.maxTurnSpeed;

    if (xSpeed < 0.2 && xSpeed > -0.2) xSpeed = 0;
    if (zRotate < 0.2 && zRotate > -0.2) zRotate = 0;

    leftMotor.set(xSpeed - zRotate);
    rightMotor.set(xSpeed + zRotate);
  }
}
