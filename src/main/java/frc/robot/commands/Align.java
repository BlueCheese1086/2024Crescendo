package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.*;

public class Align extends Command{

    private final driveSubsystem m_subsystem;
    private final boolean alignDo;

    private final RelativeEncoder encoderFL;
    private final RelativeEncoder encoderFR; 

    double yaw;
    double pitch;

    static PhotonCamera camera = new PhotonCamera("kitkam");
    static PhotonPipelineResult result = camera.getLatestResult();
    static PhotonTrackedTarget target = result.getBestTarget();
    List<PhotonTrackedTarget> targets;

    static BangBangController controller = new BangBangController();
    
    public Align(driveSubsystem subsystem, boolean alignDo) {
        m_subsystem = subsystem;
        this.alignDo = alignDo;
        encoderFL = m_subsystem.getEncoderFL();
        encoderFR = m_subsystem.getEncoderFR();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }
      
      public void execute() {
        result = camera.getLatestResult();
        if(result.hasTargets()){
          boolean targetThere = false;
          targets = result.getTargets();
          for(int i = 0; i < targets.size(); i++){
            if(targets.get(i).getFiducialId() == 4 || targets.get(i).getFiducialId() == 7){
              target = targets.get(i);
              yaw = target.getYaw();
              pitch = target.getPitch();
              targetThere = true;
            }
          }
          if (!targetThere) {
            yaw = 0;
          }
          //Transform3d camToTarget = target.getBestCameraToTarget();
          SmartDashboard.putNumber("Yaw value", yaw);
          SmartDashboard.putNumber("Pitch value", pitch);
          SmartDashboard.putNumber("Feducial ID1", targets.get(0).getFiducialId());
          if (targets.size() > 1) {
            SmartDashboard.putNumber("Feducial ID2", targets.get(1).getFiducialId());
          } else {
            SmartDashboard.putNumber("Feducial ID2", -1);
          }
          //SmartDashboard.putNumber("X distance", camToTarget.getX());
          //SmartDashboard.putNumber("Y distance", camToTarget.getY());
          //SmartDashboard.putNumber("Z distance", camToTarget.getZ());
        }
        else {
          yaw = 0;
          SmartDashboard.putNumber("Yaw value", yaw);
          SmartDashboard.putNumber("Pitch value", pitch);
        }

        if (alignDo){
            m_subsystem.driveAlign(yaw); //uncomment when you know that yaw works correctly
        } 
      }
}
