package frc.robot.subsystems;
import java.lang.invoke.ConstantCallSite;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonVersion;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;

public class shooterSubystem extends SubsystemBase {

  CANSparkMax shooterU = new CANSparkMax(ShooterConstants.UPPER_SHOOTER_ID, MotorType.kBrushless); //need actual value and motor type
  CANSparkMax shooterL = new CANSparkMax(ShooterConstants.LOWER_SHOOTER_ID, MotorType.kBrushless); //need actual value and motor type


  PhotonCamera camera = new PhotonCamera("photon vision");
  PhotonPipelineResult result = camera.getLatestResult();
  PhotonTrackedTarget target = result.getBestTarget();

  SparkPIDController lowerPID = shooterL.getPIDController();

  public shooterSubystem() {
    shooterU.restoreFactoryDefaults();
    shooterL.restoreFactoryDefaults();

    shooterU.setIdleMode(IdleMode.kCoast);
    shooterL.setIdleMode(IdleMode.kCoast);

    lowerPID.setP(0.00001);
    lowerPID.setI(0);
    lowerPID.setD(0);
    lowerPID.setFF(0.01);
  }

  @Override
  public void periodic() {
  }

  public void shootLower(boolean doShootL){
    if (doShootL){
      lowerPID.setReference(15000, ControlType.kVelocity);
    }
    else{
        lowerPID.setReference(0, ControlType.kVelocity);
    }
  } 

  public void shootUpper(boolean doShootU){
    if (doShootU){
      shooterU.set(1);
    }
    else{
        shooterU.set(0);
    }
  }

  public void intake(boolean doIntake){
    if (doIntake){
      shooterU.set(1);
      shooterL.set(1);
    }
    else{
      shooterU.set(0);
      shooterL.set(0);
    }
  }
}