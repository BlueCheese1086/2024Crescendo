package frc.robot.Intake.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Intake.Intake;
import frc.robot.Intake.Intake.States;

public class SetIntakeAngle extends Command {
    private Intake intake = Intake.getInstance();
    private Rotation2d angle;

    /**
     * Creates a new SetIntakeAngle command.
     * <p>
     * This command sets the angle of the intake to a set angle.
     * 
     * @param state An enum for the state of the intake.
     */
    public SetIntakeAngle(States state) {
        this.angle = state.value;
    }

    /** Runs every 20 ms while the command is scheduled. */
    @Override
    public void execute() {
        System.out.printf("Setting angle to %s\n", angle);
        intake.setAngle(angle);
    }
}