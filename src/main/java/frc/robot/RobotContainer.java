package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain.commands.*;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Launcher.commands.*;
import frc.robot.subsystems.Launcher.Launcher;

public class RobotContainer {
    // Defining the robot's subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final Launcher launcher = new Launcher();

    // Creating instances of the xbox remotes used for driving the robot.
    private final XboxController primaryXbox = new XboxController(OperatorConstants.PrimaryPort);
    private final XboxController secondaryXbox = new XboxController(OperatorConstants.SecondaryPort);

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        // Configuring the trigger bindings
        configureBindings();
    }

    /**
     * Creates button bindings for the necessary functions.
     */
    private void configureBindings() {
        // Creating triggers for each button.
        Trigger secondaryA = new Trigger(secondaryXbox::getAButton);
        Trigger secondaryLeftBumper = new Trigger(secondaryXbox::getLeftBumper);

        // Assigning operations to each trigger.
        // "A" runs the launcher with a bit of warmup for the launch wheel.
        // "Left Bumper" collects new notes by running the launcher in reverse.
        secondaryA.whileTrue(new RunLauncher(launcher, 1, 1, 1));
        secondaryLeftBumper.whileTrue(new RunLauncher(launcher, -1, -1, 0));
    }

    /**
     * Passes the autonomous command to the {@link Robot} class.
     *
     * @return The command to run in Autonomous mode.
     */
    public Command getAutonomousCommand() {
        return Autos.jaxAuto(drivetrain, launcher);
    }

    /**
     * Passes the teleop command to the {@link Robot} class.
     *
     * @return The command to run in Teleop mode.
     */
    public Command getTeleopCommand() {
        return new ArcadeDrive(drivetrain, () -> -primaryXbox.getLeftY(), () -> primaryXbox.getRightX());
    }
}