package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Drivetrain.commands.*;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Launcher.commands.*;
import frc.robot.subsystems.Launcher.Launcher;

/**
 * This class contains static references to different command sequences that can be used to change the autonomous program on the fly.
 */
public final class Autos {
    /**
     * Example static factory for an autonomous command.
     * <p>
     * This function returns an autonomous drive that moves the drivetrain backwards at half speed for 1 second.
     * 
     * @param drivetrain The Drivetrain subsystem that this auto sequence will pass to
     */
    public static SequentialCommandGroup exampleAuto(Drivetrain drivetrain) {
        return new SequentialCommandGroup(
            new DriveTime(drivetrain, -0.5, 0, 1)
        );
    }

    /**
     * A custom auto sequence created by Jackson.
     * <p>
     * This function returns an autonomous drive that shoots the loaded game piece, and then moves backwards at half speed for 1 second.
     */
    public static SequentialCommandGroup jaxAuto(Drivetrain drivetrain, Launcher launcher) {
        return new SequentialCommandGroup(
            new LauncherTime(launcher, 1, 1, 1, 2),
            new DriveTime(drivetrain, -0.5, 0, 1)
        );
    }
}