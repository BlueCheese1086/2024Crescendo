package frc.robot.Intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;
import frc.robot.Intake.Intake.States;

public class SetIntakeState extends Command {
    private Intake intake;
    private States state;

    public SetIntakeState(Intake intake, States state) {
        this.intake = intake;
        this.state = state;
    }

    @Override
    public void execute() {
        intake.setAccess(state);
    }
}