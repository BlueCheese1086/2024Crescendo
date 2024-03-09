package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Intake.Intake;

public class SetIntakeState extends Command {

    private final boolean rollersIn, intakeDown;

    private final Intake intake;

    public SetIntakeState(boolean rollersIn, boolean intakeDown, Intake intake) {
        this.rollersIn = rollersIn;
        this.intakeDown = intakeDown;

        this.intake = intake;

        addRequirements(intake);
    }
    
    public void initialize() {}

    public void execute() {
        intake.setRollerSpeed(rollersIn ? 3000.0 : -3000.0);
        intake.setAnglePosition(intakeDown ? 0.2 : IntakeConstants.STOWED_ANGLE);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean inter) {
        intake.stopRollers();
    }

}
