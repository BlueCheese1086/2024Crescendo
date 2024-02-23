package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;

public class RunRollers extends Command {

    private final boolean in;

    private final Intake intake;

    public RunRollers(boolean in, Intake intake) {
        this.in = in;
        this.intake = intake;

    }

    public void initialize() {}

    public void execute() {
        intake.setRollerSpeed(in ? 3000.0 : -3000.0);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interr) {
        intake.stopRollers();
    }
    
}
