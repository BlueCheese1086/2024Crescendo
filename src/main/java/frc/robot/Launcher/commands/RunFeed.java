package frc.robot.Launcher.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Launcher.Launcher;

public class RunFeed extends Command {
    private Launcher launcher;
    private double speed;

    /**
     * Creates a new RunFeed command.
     * 
     * @param launcher The {@link Launcher} subsystem to run this command on.
     * @param speed    The speed to run the motor at in percent.
     */
    public RunFeed(Launcher launcher, double speed) {
        this.launcher = launcher;
        this.speed = speed;
    }

    /** This function is called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** This function is called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        launcher.setFeedWheel(speed);
    }

    /**
     * This function returns true when the command should end.
     * It runs at the same time as the {@linkplain #execute() execute()} function.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** This function is called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        launcher.setFeedWheel(0);
    }
}