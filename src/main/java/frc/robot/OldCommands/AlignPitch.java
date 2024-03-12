package frc.robot.OldCommands;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;

public class AlignPitch extends Command{
    private final boolean alignDo;
    private final driveSubsystem m_subsystem;
    double pitch;
    static PhotonCamera camera = new PhotonCamera("kitkam");
    static PhotonPipelineResult result = camera.getLatestResult();
    static PhotonTrackedTarget target = result.getBestTarget();
    List<PhotonTrackedTarget> targets;

    public AlignPitch(driveSubsystem driveSub, boolean alignDo) {
        m_subsystem = driveSub;
        this.alignDo = alignDo;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSub);
    }
    
 public void execute() {
        SmartDashboard.putBoolean("Shift Light", false);
        result = camera.getLatestResult();
        if(result.hasTargets()){
          boolean targetThere = false;
          targets = result.getTargets();
          for(int i = 0; i < targets.size(); i++){
            if(targets.get(i).getFiducialId() == 4 || targets.get(i).getFiducialId() == 7){
              target = targets.get(i);
              pitch = target.getPitch();
              targetThere = true;
            }
          }
          if (!targetThere) {
            pitch = 0;
          }
          SmartDashboard.putNumber("Pitch value", pitch);
          SmartDashboard.putNumber("Feducial ID1", targets.get(0).getFiducialId());
          if (targets.size() > 1) {
            SmartDashboard.putNumber("Feducial ID2", targets.get(1).getFiducialId());
          } else {
            SmartDashboard.putNumber("Feducial ID2", -1);
          }
        }
        else {
          pitch = 0;
          SmartDashboard.putNumber("Pitch value", pitch);
        }

        if (alignDo){
            m_subsystem.driveAlignPitch(pitch);
        } 
      }

      @Override
      public boolean isFinished() {
        if(result.hasTargets()){
          return pitch > 2.42 && (target.getFiducialId() == 4 || target.getFiducialId() == 7);
        }
        else{
          return false;
        }
      }

      @Override
      public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shift Light", true);
        pitch = 0;
        m_subsystem.driveAlignPitch(0);
      }
}
