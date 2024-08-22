package frc.robot.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PhotonCamera flCamera;
    private PhotonCamera frCamera;

    private Transform3d robotToFLCamera = new Transform3d(new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(-12), Units.inchesToMeters(10)), new Rotation3d(5.0 / 36.0 * Math.PI, 5.0 / 36.0 * Math.PI, 7.0 / 36.0 * Math.PI));
    private Transform3d robotToFRCamera = new Transform3d(new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(-12), Units.inchesToMeters(10)), new Rotation3d(-5.0 / 36.0 * Math.PI, 5.0 / 36.0 * Math.PI, -7.0 / 36.0 * Math.PI));

    private PhotonPoseEstimator flPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToFLCamera);
    private PhotonPoseEstimator frPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToFRCamera);
    
    private VisionResult flResult;
    private VisionResult frResult;

    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }

        return instance;
    }

    public Vision() {
        flCamera = new PhotonCamera("FL_Camera");
        frCamera = new PhotonCamera("FR_Camera");
    }

    /**
     * Updates the pose estimators.
     */
    public void periodic() {
        PhotonPipelineResult newFLResult = flCamera.getLatestResult();
        Optional<EstimatedRobotPose> newFLPose = flPoseEstimator.update(newFLResult);
        if (newFLPose.isPresent()) {
            flResult.setPose(newFLPose.get().estimatedPose.toPose2d());
            flResult.setTimestamp(newFLResult.getTimestampSeconds());
        }

        PhotonPipelineResult newFRResult = frCamera.getLatestResult();
        Optional<EstimatedRobotPose> newFRPose = frPoseEstimator.update(newFRResult);
        if (newFRPose.isPresent()) {
            frResult.setPose(newFRPose.get().estimatedPose.toPose2d());
            frResult.setTimestamp(newFRResult.getTimestampSeconds());
        }
    }

    public PhotonPipelineResult getFLPhotonResult() {
        return flCamera.getLatestResult();
    }

    public PhotonPipelineResult getFRPhotonResult() {
        return frCamera.getLatestResult();
    }

    public Pose2d getFLPose() {
        return flResult.getPose();
    }

    public Pose2d getFRPose() {
        return frResult.getPose();
    }

    public VisionResult getFLPoseWithTimestamp() {
        return flResult;
    }

    public VisionResult getFRPoseWithTimestamp() {
        return frResult;
    }

    /**
     * Checks if the apriltag with the given id is visible.
     * 
     * @param id ID of the apriltag to look for.
     */
    public boolean isVisible(int id) {
        // Looping through the visible targets from the front left camera.
        for (PhotonTrackedTarget target : flCamera.getLatestResult().targets) {
            // Checking if the IDs match and returning true if they do.
            if (target.getFiducialId() == id) {
                return true;
            }
        }

        // Looping through the visible targets from the front right camera.
        for (PhotonTrackedTarget target : flCamera.getLatestResult().targets) {
            // Checking if the IDs match and returning true if they do.
            if (target.getFiducialId() == id) {
                return true;
            }
        }

        // ID not found, returning false.
        return false;
    }
}