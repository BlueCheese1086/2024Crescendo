package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;

public class RunRollerRotations extends Command {

    private final Intake intake;
    private final double distance;
    private double start;

    public RunRollerRotations(double distance, Intake intake) {
        this.intake = intake;
        this.distance = distance;
    }

    public void initialize() {
        start = intake.getRollerEncoder().getPosition();
    }

    public void execute() {
        intake.setRollerSpeed(4000.0 * Math.signum(start - distance));
    }

    public boolean isFinished() {
        return intake.getRollerEncoder().getPosition() - start >= distance;
    }

    public void end(boolean inter) {
        intake.stopRollers();
    }
    
}
