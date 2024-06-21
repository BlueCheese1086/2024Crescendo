package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Shooter;

public class SetLauncherSpeed extends Command {
    private Shooter shooter;
    private double speed;

    /**
     * Creates a new SetLauncherSpeed command.
     * <p>
     * This command runs the launcher at a set speed.
     * 
     * @param speed The percent speed to run the launcher at.
     */
    public SetLauncherSpeed(double speed) {
        this.shooter = Shooter.getInstance();
        this.speed = speed;
    }

    /** This function runs every 20 ms that this command is scheduled/. */
    @Override
    public void execute() {
        shooter.runLauncher(speed);
    }

    /** This function runs once when the command ends. */
    @Override
    public void end(boolean interrupted) {
        shooter.runLauncher(0);
    }
}