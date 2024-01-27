// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class driveSubsystem extends SubsystemBase {
  CANSparkMax rightLeader = new CANSparkMax(Constants.DriveConstants.FRONT_RIGHT_ID, MotorType.kBrushless); 
  CANSparkMax rightFollower = new CANSparkMax(Constants.DriveConstants.BACK_RIGHT_ID, MotorType.kBrushless);
  CANSparkMax leftLeader = new CANSparkMax(Constants.DriveConstants.FRONT_LEFT_ID, MotorType.kBrushless);
  CANSparkMax leftFollower = new CANSparkMax(Constants.DriveConstants.BACK_LEFT_ID, MotorType.kBrushless);

  BangBangController controller = new BangBangController();

  RelativeEncoder encoderFR = rightLeader.getEncoder();
  RelativeEncoder encoderFL = leftLeader.getEncoder();
  RelativeEncoder encoderBR = rightFollower.getEncoder();
  RelativeEncoder encoderBL = leftFollower.getEncoder();

  
  SwerveDriveKinematics kinematics;

  public driveSubsystem() {
    rightLeader.restoreFactoryDefaults();
    leftLeader.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();

    rightLeader.setIdleMode(IdleMode.kBrake);
    leftLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);

    leftLeader.setInverted(false);
    rightLeader.setInverted(true);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader); 
  }

  @Override
  public void periodic() {}

  public void set(double leftF, double rightF) {
    MathUtil.applyDeadband(leftF, Constants.DriveConstants.DEADBAND);
    MathUtil.applyDeadband(rightF, Constants.DriveConstants.DEADBAND);

    rightLeader.set(rightF);
    leftLeader.set(leftF);
    
    SmartDashboard.putNumber("Right speed", rightF);
    SmartDashboard.putNumber("Left speed", leftF);
  }

  public RelativeEncoder getEncoderFL(){
    return encoderFL;
  }

  public RelativeEncoder getEncoderFR(){
    return encoderFR;
  }

  public void driveAlign(double yaw){
      if(yaw > 0){
        leftLeader.set(controller.calculate(encoderFL.getPosition(), Math.abs(yaw)));
      }
      else {
        rightLeader.set(controller.calculate(encoderFR.getPosition(), Math.abs(yaw)));
      }
  }
}
