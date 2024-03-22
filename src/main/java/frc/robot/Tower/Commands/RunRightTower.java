package frc.robot.Tower.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Tower.Tower;

public class RunRightTower extends Command {
    private Tower tower;
    private double speed;

    /**
     * Creates a new RunRightTower command.
     * <p>
     * This command runs the right tower of the robot at the given speed.
     * 
     * @param speed The percent speed to run the right tower at.
     */
    public RunRightTower(double speed) {
        this.tower = Tower.getInstance();
        this.speed = speed;
    }

    /** This function runs every 20 ms while the command is scheduled. */
    @Override
    public void execute() {
        tower.setRightSpeed(speed);
    }

    /** This function runs once when the command ends. */
    @Override
    public void end(boolean interrupted) {
        tower.setRightSpeed(0);
    }
}