package frc.robot.Launcher.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Launcher.Launcher;

public class RunFeed extends Command {
    private Launcher launcher;
    private double speed;

    public RunFeed(Launcher launcher, double speed) {
        this.launcher = launcher;
        this.speed = speed;
    }

    @Override
    public void execute() {
        launcher.setFeedWheel(speed);
    }

    @Override
    public void end(boolean interrupted) {
        launcher.setFeedWheel(0);
    }
}