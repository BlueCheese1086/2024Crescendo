package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climb;

public class doClimb extends Command {
    public enum ClimbMode {
        UP,
        DOWN
    }

    private ClimbMode climbMode;
    
    private Climb climbSubsystem;

    public doClimb(Climb climbSub, ClimbMode mode) {
        climbSubsystem = climbSub;
        climbMode = mode;
        addRequirements(climbSub);
    }

    @Override
    public void initialize() {
        if (climbMode == ClimbMode.UP) {
            climbSubsystem.runClimb(OperatorConstants.kClimbLength);
        } else {
            climbSubsystem.runClimb(0);
        }
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
