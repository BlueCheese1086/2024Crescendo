package frc.robot.Shooter.Commands;

import frc.robot.Shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class RunShooter extends Command {
    private Shooter shooter;
    private double speed;

    public RunShooter(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
    }

    @Override
    public void execute() {
        shooter.setLaunchSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setLaunchSpeed(0);
    }
}