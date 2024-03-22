package frc.robot.Tower.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Tower.Tower;

public class RunTower extends Command {
    private Tower tower;
    private double speed;

    /**
     * Creates a new RunTower Command.
     * <p>
     * This command runs the tower of the robot at the given speed.
     * 
     * @param speed The percent speed to run the tower at.
     */
    public RunTower(double speed) {
        this.tower = Tower.getInstance();
        this.speed = speed;
    }

    /** This function runs every 20 ms while the command is scheduled. */
    @Override
    public void execute() {
        tower.setSpeed(speed);
    }

    /** This function runs once when the command ends. */
    @Override
    public void end(boolean interrupted) {
        tower.setSpeed(0);
    }
}