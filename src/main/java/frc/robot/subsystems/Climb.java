// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OperatorConstants;

public class Climb extends SubsystemBase {
    
  private final CANSparkMax leftClimb = new CANSparkMax(OperatorConstants.Kleftclimbmotorid, MotorType.kBrushless);
  private final CANSparkMax rightClimb = new CANSparkMax(OperatorConstants.Krightclimbmotorid, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftClimb.getEncoder();
  private final RelativeEncoder rightEncoder = rightClimb.getEncoder();

  private SparkPIDController climbPID = leftClimb.getPIDController();

  public Climb() 
  {
    rightClimb.follow(leftClimb);
    climbPID.setP(0.01);
    climbPID.setI(0.0);
    climbPID.setD(0.0);
  }

  public void runClimb(double climbDistance) 
  {
    stop(false);
    double error = climbDistance - getLeftDistance(); 
    climbPID.setReference(error, ControlType.kPosition);// Maybe switch to just getLeftDistance()
    stop(true);
  }

  public void stop(boolean brake) 
  {
    if (brake == true){
      leftClimb.setIdleMode(IdleMode.kBrake);
      rightClimb.setIdleMode(IdleMode.kBrake);
    } else {
      leftClimb.setIdleMode(IdleMode.kCoast);
      rightClimb.setIdleMode(IdleMode.kCoast);
    }
  }

  public double getLeftDistance() {
    return leftEncoder.getPosition();
  }
  public double getRightDistance() {
    return rightEncoder.getPosition();
  }
  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public void setP(double p) {
    climbPID.setP(p);
  }
  public void setI(double i) {
    climbPID.setI(i);
  }
  public void setD(double d) {
    climbPID.setD(d);
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
