package frc.robot.Vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.ArrayList;
import java.util.Optional;

public class Vision extends SubsystemBase {
    private PhotonCamera lCamera;
    private PhotonCamera rCamera;

    private Transform3d robotToLCamera = new Transform3d(new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(-12), Units.inchesToMeters(10)), new Rotation3d(5.0 / 36.0 * Math.PI, 5.0 / 36.0 * Math.PI, 7.0 / 36.0 * Math.PI));
    private Transform3d robotToRCamera = new Transform3d(new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(-12), Units.inchesToMeters(10)), new Rotation3d(-5.0 / 36.0 * Math.PI, 5.0 / 36.0 * Math.PI, -7.0 / 36.0 * Math.PI));

    private PhotonPoseEstimator lPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToLCamera);
    private PhotonPoseEstimator rPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToRCamera);

    private EstimatedRobotPose lPose;
    private EstimatedRobotPose rPose;

    private StructPublisher<Pose2d> lPose2dPublisher;
    private StructPublisher<Pose3d> lPose3dPublisher;
    private StructPublisher<Pose2d> rPose2dPublisher;
    private StructPublisher<Pose3d> rPose3dPublisher;
    
    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }

        return instance;
    }

    public Vision() {
        lCamera = new PhotonCamera("FL_Camera");
        rCamera = new PhotonCamera("FR_Camera");

        lPose2dPublisher = NetworkTableInstance.getDefault().getStructTopic("/Vision/Left Pose2d", Pose2d.struct).publish();
        lPose3dPublisher = NetworkTableInstance.getDefault().getStructTopic("/Vision/Left Pose3d", Pose3d.struct).publish();
        rPose2dPublisher = NetworkTableInstance.getDefault().getStructTopic("/Vision/Right Pose2d", Pose2d.struct).publish();
        rPose3dPublisher = NetworkTableInstance.getDefault().getStructTopic("/Vision/Right Pose3d", Pose3d.struct).publish();
    }

    /** Updates the pose estimators. */
    public void periodic() {
        // Updating positions
        Optional<EstimatedRobotPose> estimatedPose = lPoseEstimator.update(getLPhotonResult());

        if (!estimatedPose.isEmpty() && estimatedPose.get() != null) {
            lPose = estimatedPose.get();
        }

        estimatedPose = rPoseEstimator.update(getRPhotonResult());

        if (!estimatedPose.isEmpty() && estimatedPose.get() != null) {
            rPose = estimatedPose.get();
        }

        // Logging the visible apriltags
        ArrayList<Double> arr = new ArrayList<Double>();

        getLPhotonResult().getTargets().forEach((target) -> {
            arr.add((double) target.getFiducialId());
        });

        getRPhotonResult().getTargets().forEach((target) -> {
            arr.add((double) target.getFiducialId());
        });

        SmartDashboard.putNumberArray("/Vision/VisibleTargets", arr.toArray(new Double[0]));

        // Logging Estimated poses
        if (lPose != null) {
            lPose2dPublisher.set(lPose.estimatedPose.toPose2d());
            lPose3dPublisher.set(lPose.estimatedPose);
        }

        if (rPose != null) {
            rPose2dPublisher.set(rPose.estimatedPose.toPose2d());
            rPose3dPublisher.set(rPose.estimatedPose);
        }
    }

    public PhotonPipelineResult getLPhotonResult() {
        return lCamera.getLatestResult();
    }

    public PhotonPipelineResult getRPhotonResult() {
        return rCamera.getLatestResult();
    }

    public EstimatedRobotPose getLPose() {
        return lPose;
    }

    public EstimatedRobotPose getRPose() {
        return rPose;
    }

    /**
     * Checks if the apriltag with the given id is visible.
     * 
     * @param id ID of the apriltag to look for.
     * @return The PhotonTrackedTarget object for that apriltag.
     */
    public PhotonTrackedTarget isVisible(int id) {
        // Looping through the visible targets from the front left camera.
        for (PhotonTrackedTarget target : lCamera.getLatestResult().targets) {
            // Checking if the IDs match and returning true if they do.
            if (target.getFiducialId() == id) {
                return target;
            }
        }

        // Looping through the visible targets from the front right camera.
        for (PhotonTrackedTarget target : lCamera.getLatestResult().targets) {
            // Checking if the IDs match and returning true if they do.
            if (target.getFiducialId() == id) {
                return target;
            }
        }

        // ID not found, returning null.
        return null;
    }
}