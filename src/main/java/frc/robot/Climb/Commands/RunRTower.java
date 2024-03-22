package frc.robot.Climb.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Climb.Climb;

public class RunRTower extends Command {
    private Climb climb;
    private double speed;

    /**
     * Creates a new RunRTower command.
     * <p>
     * This command runs the right tower of the climb system at the given speed
     * 
     * @param climb The instance of the 
     * @param speed
     */
    public RunRTower(Climb climb, double speed) {
        this.climb = climb;
        this.speed = speed;
    }

    @Override
    public void execute() {
        climb.setRightSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        climb.setRightSpeed(0);
    }
}