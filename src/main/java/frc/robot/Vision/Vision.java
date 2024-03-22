package frc.robot.Vision;

import java.util.List;
import java.util.Objects;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    // Cameras
    private PhotonCamera frontCam = new PhotonCamera("Forte-Front");
    private PhotonCamera backCam = new PhotonCamera("Forte-Back");

    // Pose Estimation
    private AprilTagFieldLayout field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPoseEstimator frontPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.backCamTransform);
    private PhotonPoseEstimator backPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.backCamTransform);

    // Previous Pose
    private Pose2d frontPrevPose = new Pose2d();
    private Pose2d backPrevPose = new Pose2d();

    // A common instance of the vision subsystem.
    private static Vision instance;

    /**
     * This function gets a common instance of the vision subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Vision subsystem.
     */
    public static Vision getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (Objects.isNull(instance)) instance = new Vision();

        return instance;
    }

    /** This function runs every 20 ms that this subsystem is active. */
    public void periodic() {
        // Updating the previous poses
        frontPrevPose = getFrontPose();
        backPrevPose = getBackPose();
        
        // Updating the pose estimators
        frontPoseEstimator.update(frontCam.getLatestResult());
        backPoseEstimator.update(backCam.getLatestResult());
    }

    /**
     * Gets targets that the front camera can see.
     * 
     * @return A list of targets tracked by the front camera.
     */
    public List<PhotonTrackedTarget> getFrontTargets() {
        return frontCam.getLatestResult().targets;
    }

    /**
     * Gets targets that the back camera can see.
     * 
     * @return A list of targets tracked by the back camera.
     */
    public List<PhotonTrackedTarget> getBackTargets() {
        return backCam.getLatestResult().targets;
    }

    /**
     * Gets the estimated pose according to the front camera.
     * 
     * @return An estimated Pose2d of the robot.
     */
    public Pose2d getFrontPose() {
        return frontPoseEstimator.update().isPresent() ? frontPoseEstimator.update().get().estimatedPose.toPose2d() : frontPrevPose;
    }

    /**
     * Gets the estimated pose according to the back camera.
     * 
     * @return An estimated Pose2d of the robot.
     */
    public Pose2d getBackPose() {
        return backPoseEstimator.update().isPresent() ? backPoseEstimator.update().get().estimatedPose.toPose2d() : backPrevPose;
    }
}