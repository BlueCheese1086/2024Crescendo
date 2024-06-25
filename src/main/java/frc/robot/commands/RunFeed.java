package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunFeed extends Command {
    private Shooter shooter;
    private double speed;

    public RunFeed(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
    }

    @Override
    public void execute() {
        shooter.setFeedSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeedSpeed(0);
    }
}
