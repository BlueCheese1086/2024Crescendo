package frc.robot.Climb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Climb.Climb;

public class RunLeftClimb extends Command {
    private Climb climb;
    private double speed;

    public RunLeftClimb(Climb climb, double speed) {
        this.climb = climb;
        this.speed = speed;
    }

    @Override
    public void execute() {
        climb.setLeftSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        climb.setLeftSpeed(0);
    }
}