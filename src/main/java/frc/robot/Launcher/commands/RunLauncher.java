package frc.robot.Launcher.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Launcher.Launcher;

public class RunLauncher extends Command {
    private Launcher launcher;
    private double feedSpeed;
    private double launcherSpeed;
    private double whenLaunch;

    /**
     * Creates a new LauncherTime command.
     * 
     * @param launcher A representation of the {@link Launcher} class that this subsystem manipulates.
     * @param launcherSpeed The percent speed that the launcher wheel should run at.
     * @param feedSpeed The percent speed that the feed wheel should run at.
     * @param launchDelay The amount of time in seconds that the command should wait before running the feed wheel.
    */
    public RunLauncher(Launcher launcher, double launcherSpeed, double feedSpeed, double launchDelay) {
        this.launcher = launcher;
        this.launcherSpeed = launcherSpeed;
        this.feedSpeed = feedSpeed;
        this.whenLaunch = System.currentTimeMillis() + (launchDelay * 1000);

        addRequirements(launcher);
    }

    /** This function is called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** This function is called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        launcher.setLaunchWheel(launcherSpeed);
        if (System.currentTimeMillis() > whenLaunch) {
            launcher.setFeedWheel(feedSpeed);
        }
    }

    /** This function is called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        launcher.setSpeeds(0, 0);
    }
}