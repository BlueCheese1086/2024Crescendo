package frc.robot.Shooter.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Shooter;

public class SetShooterSpeed extends Command {
    private Shooter shooter;
    private Rotation2d angle;
    private double rate;

    /**
     * Temporary Constructor, only exists until we work out vision.
     * <p>
     * Creates a new SetShooterSpeed command.
     * <p>
     * This command moves the shooter at a rate in deg/s.
     * 
     * @param rate The rate that the shooter should move at.
     */
    public SetShooterSpeed(double rate) {
        this.shooter = Shooter.getInstance();
        this.angle = shooter.getAngle();
        this.rate = rate / 50.0;
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