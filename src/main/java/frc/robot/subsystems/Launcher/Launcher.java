package frc.robot.subsystems.Launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
    private final CANSparkMax launchWheel = new CANSparkMax(LauncherConstants.LauncherID, MotorType.kBrushless);
    private final CANSparkMax feedWheel = new CANSparkMax(LauncherConstants.FeederID, MotorType.kBrushless);

    /** Creates a new Launcher subsystem. */
    public Launcher() {
        launchWheel.setSmartCurrentLimit(LauncherConstants.LauncherCurrentLimit);
        feedWheel.setSmartCurrentLimit(LauncherConstants.LauncherCurrentLimit);
    }

    /** A mutator function that sets the speed of the launch wheel. */
    public void setLaunchWheel(double speed) {
        launchWheel.set(speed);
    }

    /** A mutator function that sets the speed of the feed wheel. */
    public void setFeedWheel(double speed) {
        feedWheel.set(speed);
    }

    /**
     * A helper function that stops both wheels.
     * <p>
     * You could skip having a method like this and call the individual accessors with speed = 0 instead.
     */
    public void stop() {
        launchWheel.set(0);
        feedWheel.set(0);
    }
}