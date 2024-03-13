package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;
import frc.robot.Intake.Intake.IntakeState;

public class AutoIntake extends Command {

    private final Intake intake;

    public AutoIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public void initialize() {}

    public void execute() {
        intake.setState(IntakeState.IntakingDown);
        intake.defaultMethod();
    }

    public boolean isFinished() {
        return intake.getIntakeSensor();
    }

    public void end(boolean interr) {}
    
}
