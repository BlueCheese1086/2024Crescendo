package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Shooter.Shooter;

public class RunShooter extends Command {

    private final double frontRPM, backRPM;

    private final Shooter shooter;

    private final CommandXboxController x;

    public RunShooter(double frontRPM, double backRPM, CommandXboxController x, Shooter shooter) {
        this.frontRPM = frontRPM;
        this.backRPM = backRPM;

        this.x = x;

        this.shooter = shooter;
        addRequirements(shooter);
    }

    public void initialize() {}

    public void execute() {
        x.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        shooter.setMotorVels(frontRPM, backRPM);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interr) {
        x.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        shooter.stopMotors();
    }
    
}
