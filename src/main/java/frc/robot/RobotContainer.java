package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Drivetrain.commands.*;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Launcher.commands.*;
import frc.robot.Launcher.Launcher;

public class RobotContainer {
    // Defining the robot's subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final Launcher launcher = new Launcher();

    // Creating instances of the xbox remotes used for driving the robot.
    private final CommandXboxController xbox = new CommandXboxController(OperatorConstants.PrimaryPort);

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        // Configuring the trigger bindings
        configureBindings();
    }

    /** Creates button bindings for the necessary functions. */
    private void configureBindings() {
        // Assigning operations to each button.
        // "Left Trigger" runs the flywheel on the launcher.
        // "Right Trigger" runs the feed wheel on the launcher.
        // "A" runs the launcher in reverse to collect notes.
        xbox.a().whileTrue(new RunFlywheel(launcher, -1));
        xbox.leftTrigger().whileTrue(new RunFlywheel(launcher, 1));
        xbox.rightTrigger().whileTrue(new RunFeed(launcher, 1).raceWith(new WaitCommand(1)));
    }

    /**
     * Passes the autonomous command to the {@link Robot} class.
     *
     * @return The command to run in Autonomous mode.
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Replace Me");
    }

    /**
     * Passes the teleop command to the {@link Robot} class.
     *
     * @return The command to run in Teleop mode.
     */
    public Command getTeleopCommand() {
        return new ArcadeDrive(drivetrain, () -> xbox.getLeftY(), () -> xbox.getRightX());
    }
}