package frc.robot.Climb.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Climb.Climb;

public class SetClimbPos extends Command {

    private final double leftPos, rightPos;

    private final Climb climb;

    public SetClimbPos(double leftPos, double rightPos, Climb climb) {
        this.leftPos = leftPos;
        this.rightPos = rightPos;

        this.climb = climb;
        // addRequirements(climb);
    }

    public void initialize() {}

    public void execute() {
        climb.setPosition(leftPos, rightPos);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interr) {
        climb.stopMotors();
    }
    
}