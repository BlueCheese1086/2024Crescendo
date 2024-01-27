package frc.robot.subsystems;
import java.lang.invoke.ConstantCallSite;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonVersion;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;

public class shooterSubystem extends SubsystemBase {

  CANSparkMax shooterU = new CANSparkMax(Constants.ShooterConstants.UPPER_SHOOTER_ID, MotorType.kBrushless); //need actual value and motor type
  CANSparkMax shooterL = new CANSparkMax(Constants.ShooterConstants.LOWER_SHOOTER_ID, MotorType.kBrushless); //need actual value and motor type


  PhotonCamera camera = new PhotonCamera("photon vision");
  PhotonPipelineResult result = camera.getLatestResult();
  PhotonTrackedTarget target = result.getBestTarget();

  PIDController alignPID = new PIDController(1, 0, 0);

  public shooterSubystem() {
    shooterU.restoreFactoryDefaults();
    shooterL.restoreFactoryDefaults();

    shooterU.setIdleMode(IdleMode.kCoast);
    shooterL.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void shoot(boolean doShoot){
    if (doShoot){
      shooterU.set(Constants.ShooterConstants.SHOOTER_MAX_SPEED);
      shooterL.set(Constants.ShooterConstants.SHOOTER_MAX_SPEED * 0.8);
    }
    else{
        shooterU.set(0);
        shooterL.set(0);
    }
    SmartDashboard.putBoolean("Shooter on?", doShoot);
  } 
}