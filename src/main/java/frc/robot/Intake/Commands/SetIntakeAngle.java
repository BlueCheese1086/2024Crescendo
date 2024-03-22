package frc.robot.Intake.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Intake.Intake;

public class SetIntakeAngle extends Command {
    private Intake intake;
    private Rotation2d angle;

    /**
     * Creates a new SetIntakeAngle command.
     * <p>
     * This command sets the angle of the intake to a set angle.
     * 
     * @param angle The angle of the intake.
     */
    public SetIntakeAngle(Rotation2d angle) {
        this.intake = Intake.getInstance();
        this.angle = angle;
    }

    /** Runs every 20 ms while the command is scheduled. */
    @Override
    public void execute() {
        intake.setAngle(angle);
    }

    /** Runs once when the command ends. */
    @Override
    public void end(boolean interrupted) {
        intake.setAngle(new Rotation2d());
    }
}