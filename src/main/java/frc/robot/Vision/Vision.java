package frc.robot.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PhotonCamera mainCam = new PhotonCamera("kitkam");

    public Vision() {
        PhotonPipelineResult targets = mainCam.getLatestResult();
    }

    /**
     * This method runs every tick (20ms)
     */
    @Override
    public void periodic() {
        
    }
}