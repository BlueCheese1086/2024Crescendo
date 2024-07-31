package frc.robot.Drivetrain.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.Drivetrain;

public class ResetGyro extends Command {
    private Drivetrain drivetrain;

    public ResetGyro() {
        this.drivetrain = Drivetrain.getInstance();
    }

    public void execute() {
        drivetrain.setAngle(new Rotation2d(0));
    }

    public boolean isFinished() {
        return true;
    }
}
