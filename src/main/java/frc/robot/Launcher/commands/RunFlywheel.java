package frc.robot.Launcher.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Launcher.Launcher;

public class RunFlywheel extends Command {
    private Launcher launcher;
    private double speed;

    /**
     * Creates a new RunFlywheel command.
     * 
     * @param launcher A representation of the {@link Launcher} class that this subsystem manipulates.
     * @param speed    The percent speed that the launcher wheel should run at.
     */
    public RunFlywheel(Launcher launcher, double speed) {
        this.launcher = launcher;
        this.speed = speed;
    }

    /** This function is called when the command is initially scheduled. */
    @Override
    public void initialize() {
    }

    /** This function is called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        launcher.setLaunchWheel(speed);
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
        launcher.setLaunchWheel(0);
    }
}