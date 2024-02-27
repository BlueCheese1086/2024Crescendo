package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Intake.Intake;

public class IntakeForAmp extends ParallelCommandGroup {

    public IntakeForAmp(Intake intake) {
        addCommands(
            new SetAngle(0.5, intake),
            new RunRollerRotations(10, intake)
        );
    }
    
}