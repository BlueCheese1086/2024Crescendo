package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Intake.Intake;

public class SetIntakeSpeed extends Command {
    private Intake intake = Intake.getInstance();
    private double speed;

    /**
     * Creates a new SetIntakeSpeed command.
     * <p>
     * This command runs the rollers of the intake at a set speed.
     * 
     * @param speed The percent speed of the rollers.
     */
    public SetIntakeSpeed(double speed) {
        this.speed = speed;
    }

    /** Runs every 20 ms while the command is scheduled. */
    @Override
    public void execute() {
        intake.setSpeed(speed);
    }

    /** Runs once when the command ends. */
    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
    }
}