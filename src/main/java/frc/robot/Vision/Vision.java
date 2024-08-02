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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PhotonCamera flCamera;
    private PhotonCamera frCamera;

    private Transform3d robotToFLCamera = new Transform3d(new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(-12), Units.inchesToMeters(10)), new Rotation3d(5.0 / 36.0 * Math.PI, 5.0 / 36.0 * Math.PI, 7.0 / 36.0 * Math.PI));
    private Transform3d robotToFRCamera = new Transform3d(new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(-12), Units.inchesToMeters(10)), new Rotation3d(-5.0 / 36.0 * Math.PI, 5.0 / 36.0 * Math.PI, -7.0 / 36.0 * Math.PI));

    private PhotonPoseEstimator flPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToFLCamera);
    private PhotonPoseEstimator frPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToFRCamera);
    
    public Vision() {
        flCamera = new PhotonCamera("FL_Camera");
        frCamera = new PhotonCamera("FR_Camera");
    }

    public void getResults() {
        PhotonPipelineResult result1 = flCamera.getLatestResult();
        PhotonPipelineResult result2 = frCamera.getLatestResult();

        

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
