// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OperatorConstants;

public class Climb extends SubsystemBase {
    
        private final CANSparkMax leftClimb = new CANSparkMax(OperatorConstants.Kleftclimbmotorid, MotorType.kBrushless);
    private final CANSparkMax rightClimb = new CANSparkMax(OperatorConstants.Krightclimbmotorid, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftClimb.getEncoder();
    private final RelativeEncoder rightEncoder = rightClimb.getEncoder();
  /** Creates a new ExampleSubsystem. */
  public Climb() 
  {
    rightClimb.follow(leftClimb);
  }

  public void setSpeed(double speed) 
  {
    leftClimb.set(speed);
  }
public void stop(boolean brake) 
  {
    if (brake == true){
    leftClimb.setIdleMode(IdleMode.kBrake);
    rightClimb.setIdleMode(IdleMode.kBrake);
    }
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
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
