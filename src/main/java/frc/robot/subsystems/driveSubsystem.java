// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final static CANSparkMax rightLeader = new CANSparkMax(2, MotorType.kBrushless); 
  private final static CANSparkMax rightFollower = new CANSparkMax(4, MotorType.kBrushless);
  private final static CANSparkMax leftLeader = new CANSparkMax(1, MotorType.kBrushless);
  private final static CANSparkMax leftFollower = new CANSparkMax(3, MotorType.kBrushless);

  private final static BangBangController controller = new BangBangController();

  private final static RelativeEncoder encoderFR = rightLeader.getEncoder();
  private final static RelativeEncoder encoderFL = leftLeader.getEncoder();
  private final static RelativeEncoder encoderBR = rightFollower.getEncoder();
  private final static RelativeEncoder encoderBL = leftFollower.getEncoder();

  
  SwerveDriveKinematics kinematics;

  public driveSubsystem() {
    //ChassisSpeeds speeds = new ChassisSpeeds(0.0001, 0, 0);

    // Convert to module states
    //SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    //SwerveModuleState frontLeft = moduleStates[0];

    // Front right module state
    //SwerveModuleState frontRight = moduleStates[1];

    // Back left module state
    //SwerveModuleState backLeft = moduleStates[2];

    // Back right module state
    //SwerveModuleState backRight = moduleStates[3]; 
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double leftF, double rightF, double leftB, double rightB) {
    leftLeader.set(leftF);
    rightLeader.set(rightF);
  }

  public static RelativeEncoder getEncoderFL(){
    return encoderFL;
  }

  public static RelativeEncoder getEncoderFR(){
    return encoderFR;
  }

  public void driveAlign(double pitch){
      if(pitch > 0){
        leftLeader.set(controller.calculate(encoderFL.getPosition(), Math.abs(pitch)));
      }
      else {
        rightLeader.set(controller.calculate(encoderFR.getPosition(), Math.abs(pitch)));
      }
  }
  
}
