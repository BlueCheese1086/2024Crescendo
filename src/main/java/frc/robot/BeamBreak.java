package frc.robot;

import java.util.Objects;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeamConstants;

public class BeamBreak extends SubsystemBase {
    // Sensors
    private DigitalInput shooter = new DigitalInput(BeamConstants.shooterID);
    private DigitalInput feed = new DigitalInput(BeamConstants.feedID);

    // A common instance of the beam break subsystem
    private static BeamBreak instance;

    /**
     * This function gets a common instance of the beam break subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the BeamBreak subsystem.
     */
    public static BeamBreak getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (Objects.isNull(instance)) instance = new BeamBreak();

        return instance;
    }

    /**
     * Gets the value of the shooter beam break.
     * <p>
     * If true, then the robot does not have a note in the shooter, or there are power issues.
     * <p>
     * If false, then the robot is ready to shoot.
     * 
     * @return The value of the shooter beam break.
     */
    public boolean getShooter() {
        return shooter.get();
    }

    /**
     * Gets the value of the feed beam break.
     * <p>
     * If false, then the robot does not have a note in the feed, or there are power issues.
     * <p>
     * If false, then the note is partially in the shooter.
     * 
     * @return
     */
    public boolean getFeed() {
        return feed.get();
    }
}