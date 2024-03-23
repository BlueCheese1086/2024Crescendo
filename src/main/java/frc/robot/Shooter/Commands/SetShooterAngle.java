package frc.robot.Shooter.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Shooter;

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
    public SetShooterAngle(Rotation2d angle) {
        this.shooter = Shooter.getInstance();
        this.angle = angle;
    }

    /**
     * Temporary Constructor, only exists until we work out vision.
     * <p>
     * Creates a new SetShooterAngle command.
     * <p>
     * This command moves the shooter at a rate in deg/s.
     * 
     * @param angle The initial angle of the shooter.
     * @param rate The rate that the shooter should move at.
     */
    public SetShooterAngle(Rotation2d angle, double rate) {
        this.shooter = Shooter.getInstance();
        this.angle = angle;
        this.rate = rate / 50;
    }

    /** This function runs every 20 ms that this command is scheduled/. */
    @Override
    public void execute() {
        shooter.setAngle(angle.plus(Rotation2d.fromDegrees(rate)));
    }

    /** This function runs once when the command ends. */
    @Override
    public void end(boolean interrupted) {
        shooter.setAngle(angle);
    }
}