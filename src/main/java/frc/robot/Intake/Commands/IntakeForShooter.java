package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Intake.Intake;

public class IntakeForShooter extends ParallelCommandGroup {

    public IntakeForShooter(Intake intake) {
        addCommands(
            new SetAngle(0, intake),
            new RunRollers(true, intake)
        );
    }
    
}
