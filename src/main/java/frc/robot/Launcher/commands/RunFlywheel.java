package frc.robot.Launcher.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Launcher.Launcher;

public class RunFlywheel extends Command {
    private Launcher launcher;
    private double speed;

    /** Creates a new RunFlywheel command. 
     * @param launcher A representation of the {@link Launcher} class that this subsystem manipulates.
     * @param speed The percent speed that the launcher wheel should run at.
    */
    public RunFlywheel(Launcher launcher, double speed) {
        this.launcher = launcher;
        this.speed = speed;
    }

    /**
     * This function is called every time the scheduler runs while the command is scheduled.
     * 
     * Runs the launch wheel for the entire duration of the command.
     */
    @Override
    public void execute() {
        launcher.setLaunchWheel(speed);
    }

    /** This function is called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        launcher.setLaunchWheel(0);
    }
}