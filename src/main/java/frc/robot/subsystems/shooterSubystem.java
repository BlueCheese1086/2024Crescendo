package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonVersion;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;

public class shooterSubystem extends SubsystemBase {

  static CANSparkMax shooterU = new CANSparkMax(22, MotorType.kBrushless); //need actual value and motor type
  static CANSparkMax shooterL = new CANSparkMax(21, MotorType.kBrushless); //need actual value and motor type


  static PhotonCamera camera = new PhotonCamera("photon vision");
  static PhotonPipelineResult result = camera.getLatestResult();
  static PhotonTrackedTarget target = result.getBestTarget();

  static PIDController alignPID = new PIDController(1, 0, 0);



  /** Creates a new ExampleSubsystem. */
  public shooterSubystem() {
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static void shoot(boolean doShoot){
    if (doShoot){
      shooterU.set(1);
      shooterL.set(0.8*1);
    }
    else{
        shooterU.set(0);
        shooterL.set(0);
    }
  } 

}