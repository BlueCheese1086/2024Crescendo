package frc.robot.OldCommands;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.Constants.DriveConstants;

public class AlignYaw extends Command{

    private final driveSubsystem m_subsystem;
    private final boolean alignDo;

    double yaw;
    double pitch;

    static PhotonCamera camera = new PhotonCamera("kitkam");
    static PhotonPipelineResult result = camera.getLatestResult();
    static PhotonTrackedTarget target = result.getBestTarget();
    List<PhotonTrackedTarget> targets;

    static BangBangController controller = new BangBangController();
    
    public AlignYaw(driveSubsystem subsystem, boolean alignDo) {
        m_subsystem = subsystem;
        this.alignDo = alignDo;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
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
              yaw = target.getYaw();
              pitch = target.getPitch();
              targetThere = true;
              
            }
          }
          if (!targetThere) {
            yaw = 0;
          }
          SmartDashboard.putNumber("Yaw value", yaw);
          SmartDashboard.putNumber("Pitch value", pitch);
          //Transform3d camToTarget = target.getBestCameraToTarget();
          //SmartDashboard.putNumber("X distance", camToTarget.getX());
          //SmartDashboard.putNumber("Y distance", camToTarget.getY());
          //SmartDashboard.putNumber("Z distance", camToTarget.getZ());
          //SmartDashboard.putNumber("Feducial ID1", targets.get(0).getFiducialId());
          /*if (targets.size() > 1) {
            SmartDashboard.putNumber("Feducial ID2", targets.get(1).getFiducialId());
          } else {
            SmartDashboard.putNumber("Feducial ID2", -1);
          }*/
        }
        else {
          yaw = 0;
          SmartDashboard.putNumber("Yaw value", yaw);
        }

        if (alignDo){
            m_subsystem.driveAlignYaw(yaw);
        } 
      }

      @Override
      public boolean isFinished() {
        if(result.hasTargets()){
          return MathUtil.applyDeadband(yaw, DriveConstants.YAW_DEADBAND) == 0 && (target.getFiducialId() == 4 || target.getFiducialId() == 7);
        }
        else{
          return false;
        }
      }

      @Override
      public void end(boolean interrupted) {
        if(pitch > 2.42){
          SmartDashboard.putBoolean("Shift Light", true);
        }
        else{
          SmartDashboard.putBoolean("Shift Light", false);
        }
        yaw = 0;
        m_subsystem.driveAlignYaw(0);
      }
}
