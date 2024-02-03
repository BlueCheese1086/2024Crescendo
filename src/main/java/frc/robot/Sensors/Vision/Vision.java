package frc.robot.Sensors.Vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PhotonCamera mainCam = new PhotonCamera("MainCam");

    public Vision() {
        
    }
}