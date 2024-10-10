package frc.robot.Shooter.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Shooter.Positions;

public class SetShooterAngle extends Command {
    private Shooter shooter;
    private Rotation2d angle;
    private double rate;

    /**
     * Creates a new SetShooterAngle command.
     * <p>
     * This command sets the angle of the shooter to a set angle.
     * 
     * @param angle The angle to set the shooter to.
     */
    public SetShooterAngle(Positions angle) {
        this.shooter = Shooter.getInstance();
        this.angle = angle.value;
    }

    /** This function runs every 20 ms that this command is scheduled/. */
    @Override
    public void execute() {
        shooter.setAngle(angle.plus(Rotation2d.fromDegrees(rate)));
    }

    /** This function runs once when the command ends. */
    @Override
    public void end(boolean interrupted) {}
}