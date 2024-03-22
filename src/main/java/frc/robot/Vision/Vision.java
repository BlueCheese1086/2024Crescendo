package frc.robot.Vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    // Cameras
    public static PhotonCamera frontCam = new PhotonCamera("Forte-Front");
    public static PhotonCamera backCam = new PhotonCamera("Forte-Back");

    // Pose Estimation
    public static AprilTagFieldLayout field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static PhotonPoseEstimator frontPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.backCamTransform);
    public static PhotonPoseEstimator backPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.backCamTransform);

    // Previous Pose
    public Pose2d frontPrevPose = new Pose2d();
    public Pose2d backPrevPose = new Pose2d();

    public Vision() {}

    public void periodic() {
        frontPrevPose = getFrontPose();
        backPrevPose = getBackPose();
        
        frontPoseEstimator.update(frontCam.getLatestResult());
        backPoseEstimator.update(backCam.getLatestResult());
    }

    public List<PhotonTrackedTarget> getFrontTargets() {
        return frontCam.getLatestResult().targets;
    }

    public List<PhotonTrackedTarget> getBackTargets() {
        return backCam.getLatestResult().targets;
    }

    public Pose2d getFrontPose() {
        return frontPoseEstimator.update().isPresent() ? frontPoseEstimator.update().get().estimatedPose.toPose2d() : frontPrevPose;
    }

    public Pose2d getBackPose() {
        return backPoseEstimator.update().isPresent() ? backPoseEstimator.update().get().estimatedPose.toPose2d() : backPrevPose;
    }
}