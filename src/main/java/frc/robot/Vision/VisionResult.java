package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A small class that simply stores a Pose2d and a timestamp in the same place.
 * This allows me to return one object whenever the drivetrain wants to add a vision estimate.
 */
public class VisionResult {
    private Pose2d pose;
    private double timestamp;

    public VisionResult(Pose2d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }

    /**
     * Gets the pose.
     * 
     * @return The current pose as a {@link Pose2d}.
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Sets the pose.
     * 
     * @param pose The new pose as a {@link Pose2d}.
     */
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    /**
     * Gets the timestamp.
     * 
     * @return The current timestamp as a double.
     */
    public double getTimestamp() {
        return timestamp;
    }

    /**
     * Sets the timestamp.
     * 
     * @param timestamp The new timestamp as a double.
     */
    public void setTimestamp(double timestamp) {
        this.timestamp = timestamp;
    }
}