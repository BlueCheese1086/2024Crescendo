package frc.robot.OldCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;

public class AutoShoot extends SequentialCommandGroup{

    public AutoShoot(driveSubsystem driveSub, shooterSubystem shootSub){
        addCommands(
            new AlignYaw(driveSub, true),
            new AlignPitch(driveSub, true),
            new AlignYaw(driveSub, true),
            new AlignPitch(driveSub, true),
            new AlignYaw(driveSub, true),
            new Shoot(shootSub)
        );
    }
}

