package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Shooter;

public class SetFeedSpeed extends Command {
    private Shooter shooter;
    private double speed;

    /**
     * Creates a new SetFeedSpeed command.
     * <p>
     * This command runs the feed at a set speed.
     * 
     * @param speed The percent speed to run the feed at.
     */
    public SetFeedSpeed(double speed) {
        this.shooter = Shooter.getInstance();
        this.speed = speed;
    }

    /** This function runs every 20 ms that this command is scheduled/. */
    @Override
    public void execute() {
        shooter.runFeed(speed);
    }

    /** This function runs once when the command ends. */
    @Override
    public void end(boolean interrupted) {
        shooter.runFeed(0);
    }
}

// Note for Ross: 4.063