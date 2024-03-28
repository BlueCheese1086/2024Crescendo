package frc.robot.Vision;

import java.util.List;
import java.util.Objects;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    // Camera
    private PhotonCamera camera = new PhotonCamera("Forte");

    // Pose Estimation
    private AprilTagFieldLayout field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(field, PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.camTransform);

    // Previous Pose
    private Pose2d prevPose = new Pose2d();

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
        prevPose = getPose();
        
        // Updating the pose estimators
        poseEstimator.update(camera.getLatestResult());
    }

    /**
     * Gets targets that the camera can see.
     * 
     * @return A list of targets tracked by the camera.
     */
    public List<PhotonTrackedTarget> getTargets() {
        return camera.getLatestResult().targets;
    }

    /**
     * Gets the estimated pose according to the camera.
     * 
     * @return An estimated Pose2d of the robot.
     */
    public Pose2d getPose() {
        return poseEstimator.update().isPresent() ? poseEstimator.update().get().estimatedPose.toPose2d() : prevPose;
    }
}