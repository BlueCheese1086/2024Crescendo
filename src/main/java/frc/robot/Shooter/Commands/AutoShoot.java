package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;

public class AutoShoot extends Command {

    private final Shooter shooter;
    private final Intake intake;

    private boolean end = false;

    public AutoShoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;

        addRequirements(shooter);
    }
    
    public void initialize() {
        end = false;
        // if (!intake.getIntakeSensor()) end = true;
    }

    public void execute() {
        if (!intake.getShooterSensor()) return;
        shooter.setMotorVels(5500, shooter.getFrontRPM() > 5000.0 ? 15000 : 0.0);
        end = true;
    }

    public boolean isFinished() {
        return end && !intake.getShooterSensor();
    }

    public void end(boolean interr) {
        shooter.stopMotors();
    }

}
