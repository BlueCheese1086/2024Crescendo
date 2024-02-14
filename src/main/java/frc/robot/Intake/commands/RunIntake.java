package frc.robot.Intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;

public class RunIntake extends Command {
    private Intake intake;
    private double speed;

    public RunIntake(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void execute() {
        intake.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
    }
}