package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Shooter;

public class RunShooter extends Command {

    private final double frontRPM, backRPM;

    private final Shooter shooter;

    public RunShooter(double frontRPM, double backRPM, Shooter shooter) {
        this.frontRPM = frontRPM;
        this.backRPM = backRPM;

        this.shooter = shooter;
        addRequirements(shooter);
    }

    public void initialize() {}

    public void execute() {
        shooter.setMotorVels(frontRPM, backRPM);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interr) {
        shooter.stopMotors();
    }
    
}
