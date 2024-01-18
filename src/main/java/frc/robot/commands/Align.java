package frc.robot.commands;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.*;

public class Align extends CommandBase{

    private final driveSubsystem m_subsystem;
    private final BooleanSupplier alignDo;

    private final RelativeEncoder encoderFL = driveSubsystem.getEncoderFL();
    private final RelativeEncoder encoderFR = driveSubsystem.getEncoderFR(); 

    double yaw;
    double pitch;

    static PhotonCamera camera = new PhotonCamera("photon vision");
    static PhotonPipelineResult result = camera.getLatestResult();
    static PhotonTrackedTarget target = result.getBestTarget();

    static BangBangController controller = new BangBangController();
    
    public Align(driveSubsystem subsystem, BooleanSupplier alignDo) {
        m_subsystem = subsystem;
        this.alignDo = alignDo;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }
      
      public void execute() {
        result = camera.getLatestResult();
        target = result.getBestTarget();

        if(result.hasTargets()){
            target = result.getBestTarget();
      
            yaw = target.getYaw();
            pitch = target.getPitch();
            Transform3d camToTarget = target.getBestCameraToTarget();
        }

        if (alignDo.getAsBoolean()){
            m_subsystem.driveAlign(yaw);
        }
      }
}
