package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.commands.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;

public class RobotContainer {
    // Defining the robot's subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    // private final Launcher launcher = new Launcher();
    // private final Climb climb = new Climb();
    // private final Intake intake = new Intake();
    // private fina Pigeon2 gyro = new Pigeon2();

    // Creating instances of the xbox remotes used for driving the robot.
    private final CommandXboxController xbox = new CommandXboxController(0);

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
        // xbox.a().whileTrue(new RunFlywheel(launcher, -1));
        // xbox.leftTrigger().whileTrue(new RunFlywheel(launcher, 1));
        // xbox.rightTrigger().whileTrue(new RunFeed(launcher, 1).raceWith(new WaitCommand(1)));
    }

    /**
     * Passes the autonomous command to the {@link Robot} class.
     *
     * @return The command to run in Autonomous mode.
     */
    public Command getAutonomousCommand() {
        return null;//Autos.mainAuto(drivetrain, launcher);
    }

    /**
     * Passes the teleop command to the {@link Robot} class.
     *
     * @return The command to run in Teleop mode.
     */
    public Command getTeleopCommand() {
        return new SwerveDrive(drivetrain,
            () -> MathUtil.applyDeadband( xbox.getLeftX(), DriveConstants.deadband),
            () -> MathUtil.applyDeadband( xbox.getLeftY(), DriveConstants.deadband),
            () -> MathUtil.applyDeadband(xbox.getRightX(), DriveConstants.deadband)
        );
    }
}