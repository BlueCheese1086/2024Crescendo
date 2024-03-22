package frc.robot.Shooter.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Shooter;

public class SetShooterAngle extends Command {
    private Shooter shooter;
    private Rotation2d angle;

    /**
     * Creates a new SetShooterAngle command.
     * <p>
     * This command sets the angle of the shooter to a set angle.
     * 
     * @param angle The angle to set the shooter to.
     */
    public SetShooterAngle(Rotation2d angle) {
        this.shooter = Shooter.getInstance();
        this.angle = angle;
    }

    /** This function runs every 20 ms that this command is scheduled/. */
    @Override
    public void execute() {
        shooter.setAngle(angle);
    }

    /** This function runs once when the command ends. */
    @Override
    public void end(boolean interrupted) {
        shooter.setAngle(angle);
    }
}