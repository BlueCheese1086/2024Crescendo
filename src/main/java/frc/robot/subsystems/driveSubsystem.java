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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  CommandXboxController xbox = new CommandXboxController(0);
  XboxController joy = xbox.getHID();
  
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

    leftLeader.setSmartCurrentLimit(Constants.DriveConstants.DRIVETRAINLIMITS);
    leftFollower.setSmartCurrentLimit(Constants.DriveConstants.DRIVETRAINLIMITS);
    rightLeader.setSmartCurrentLimit(Constants.DriveConstants.DRIVETRAINLIMITS);
    rightFollower.setSmartCurrentLimit(Constants.DriveConstants.DRIVETRAINLIMITS);

    leftLeader.setInverted(false);
    rightLeader.setInverted(true);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader); 
  }

  @Override
  public void periodic() {}

  public void set(double rotateSpeed, double driveSpeed) {
    //MathUtil.applyDeadband(leftF, Constants.DriveConstants.DEADBAND);
    //MathUtil.applyDeadband(rightF, Constants.DriveConstants.DEADBAND);

    rightLeader.set((driveSpeed + rotateSpeed));
    leftLeader.set((driveSpeed - rotateSpeed));
    
    SmartDashboard.putNumber("Rotate speed:", rotateSpeed);
    SmartDashboard.putNumber("drive speed:", driveSpeed);
    SmartDashboard.putNumber("Joystick x", joy.getRightX());
    SmartDashboard.putNumber("Joystick y", joy.getRightY());
  }

  public RelativeEncoder getEncoderFL(){
    return encoderFL;
  }

  public RelativeEncoder getEncoderFR(){
    return encoderFR;
  }

  public void driveAlign(double yaw){
      if(yaw > 0){
        leftLeader.set(Math.abs(controller.calculate(yaw, 0) * 0.1));
      }
      else {
        rightLeader.set(Math.abs(controller.calculate(yaw, 0) * 0.1));
      }
  }
}
