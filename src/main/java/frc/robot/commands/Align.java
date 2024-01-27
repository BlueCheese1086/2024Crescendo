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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.*;

public class Align extends Command{

    private final driveSubsystem m_subsystem;
    private final BooleanSupplier alignDo;

    private final RelativeEncoder encoderFL;
    private final RelativeEncoder encoderFR; 

    double yaw;
    double pitch;

    static PhotonCamera camera = new PhotonCamera("photon vision");
    static PhotonPipelineResult result = camera.getLatestResult();
    static PhotonTrackedTarget target = result.getBestTarget();

    static BangBangController controller = new BangBangController();
    
    public Align(driveSubsystem subsystem, BooleanSupplier alignDo) {
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
            target = result.getBestTarget();
      
            yaw = target.getYaw();
            pitch = target.getPitch();
            Transform3d camToTarget = target.getBestCameraToTarget();
            SmartDashboard.putNumber("Yaw value", yaw);
            SmartDashboard.putNumber("X distance", camToTarget.getX());
            SmartDashboard.putNumber("Y distance", camToTarget.getY());
            SmartDashboard.putNumber("Z distance", camToTarget.getZ());
        }

        if (alignDo.getAsBoolean()){
            //m_subsystem.driveAlign(yaw); //uncomment when you know that yaw works correctly and conversion is set
        }
      }
}
