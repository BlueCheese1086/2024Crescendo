package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;

public class IntakeDefault extends Command {

    private final Intake intake;

    public IntakeDefault(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public void initialize() {}

    public void execute() {
        intake.defaultMethod();
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interr) {}
    
}
