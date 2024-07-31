package frc.robot.Vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PhotonCamera FLCamera;
    private PhotonCamera FRCamera;

    private Transform3d robotToFLCamera = new Transform3d(new Translation3d(), new Rotation3d());
    private Transform3d robotToFRCamera = new Transform3d(new Translation3d(), new Rotation3d());

    private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToFLCamera);
    
    public Vision() {
        FLCamera = new PhotonCamera("FL_Camera");
        FRCamera = new PhotonCamera("FR_Camera");
    }

    public void getResults() {
        PhotonPipelineResult result1 = FLCamera.getLatestResult();
        PhotonPipelineResult result2 = FRCamera.getLatestResult();

        if (result1.hasTargets()) {
            List<PhotonTrackedTarget> targets = result1.targets;

            for (PhotonTrackedTarget target : targets) {
                boolean onBlue = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);

                if (onBlue) {
                    switch (target.getFiducialId()) {

                    }
                } else {

                }

                switch(target.getFiducialId()) {
                    case 1:
                        break;
                    case 2:
                        break;
                    case 3:
                        break;
                    case 4:
                        break;
                    case 5:
                        break;
                    case 6:
                        break;
                    case 7:
                        break;
                    case 8:
                        break;
                    case 9:
                        break;
                    case 10:
                        break;
                    case 11:
                        break;
                    case 12:
                        break;
                    case 13:
                        break;
                    case 14:
                        break;
                    case 15:
                        break;
                    case 16:
                        break;
                }
            }
        }

        if (result2.hasTargets()) {
            List<PhotonTrackedTarget> targets = result2.targets;
        }
    }
}
