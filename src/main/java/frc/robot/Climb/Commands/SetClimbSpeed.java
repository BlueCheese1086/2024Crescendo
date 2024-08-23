package frc.robot.Climb.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Climb.Climb;

public class SetClimbSpeed extends Command {
    private Climb climb;
    private double speed;

    /**
     * Creates a new RunTower Command.
     * <p>
     * This command runs the tower of the robot at the given speed.
     * 
     * @param speed The percent speed to run the tower at.
     */
    public SetClimbSpeed(double speed) {
        this.climb = Climb.getInstance();
        this.speed = speed;
    }

    /** This function runs every 20 ms while the command is scheduled. */
    @Override
    public void execute() {
        climb.setSpeed(speed);
    }

    /**
     * This function runs every 20 ms while the command is scheduled.
     * 
     * @return Returns true when the command should end.
     */
    @Override
    public boolean isFinished() {
        return climb.atLimits();
    }

    /** This function runs once when the command ends. */
    @Override
    public void end(boolean interrupted) {
        climb.setSpeed(0);
    }
}