package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;
import frc.robot.Intake.Intake.IntakeState;

public class SetIntakeState extends Command {

    private final IntakeState state;

    private final Intake intake;

    public SetIntakeState(IntakeState state, Intake intake) {

        this.state = state;

        this.intake = intake;
        // addRequirements(intake);
    }
    
    public void initialize() {}

    public void execute() {
        intake.setState(state);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean inter) {
        // intake.setState(IntakeState.IdlingUp);
    }

}
