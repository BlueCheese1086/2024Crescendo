package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;

public class SetAngle extends Command {

    private final double angle;

    private final Intake intake;

    public SetAngle(double angle, Intake intake) {
        this.angle = angle;
        this.intake = intake;

    }

    public void initialize() {}

    public void execute() {
        intake.setAnglePosition(angle);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interr) {
        intake.stopAngle();
    }
    
}
