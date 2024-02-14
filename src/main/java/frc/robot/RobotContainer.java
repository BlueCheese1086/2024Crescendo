package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.commands.*;
import frc.robot.Launcher.Launcher;
import frc.robot.Launcher.commands.*;
import frc.robot.Climb.Climb;
import frc.robot.Climb.commands.*;
import frc.robot.Intake.Intake;
import frc.robot.Intake.commands.*;
import frc.robot.Intake.Intake.States;
import frc.robot.Constants.*;

public class RobotContainer {
    // Defining the robot's subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final Launcher launcher = new Launcher();
    private final Climb climb = new Climb();
    private final Intake intake = new Intake();
    // private fina Pigeon2 gyro = new Pigeon2();

    // Creating instances of the xbox remotes used for driving the robot.
    private final CommandXboxController xbox = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        // Configuring the NamedCommands for PathPlanner
        NamedCommands.registerCommand("ShootNote", new RunLauncher(launcher, 1, 3, 0.5));
        NamedCommands.registerCommand("RunIntake", new RunIntake(intake, 1));
        NamedCommands.registerCommand("OpenIntake", new SetIntakeState(intake, States.OPEN));
        NamedCommands.registerCommand("CloseIntake", new SetIntakeState(intake, States.CLOSED));

        // Configuring the trigger bindings
        configureBindings();
    }

    /** Creates button bindings for the necessary functions. */
    private void configureBindings() {
        // Assigning operations to each button.
        // A runs the intake to pick up notes.
        // B runs the intake in reverse.
        // X toggles the flywheel.
        // Y runs the feed wheel while racing with a 1 second wait command.
        // Left Bumper moves the left tower up.
        // Left Trigger moves the left tower down.
        // Right Bumper moves the right tower up.
        // Right Trigger moves the right tower down.
        // POV Up opens the intake.
        // POV down closes the intake.
        xbox.a().whileTrue(new RunIntake(intake, 1));
        xbox.b().whileTrue(new RunIntake(intake, -1));
        xbox.x().toggleOnTrue(new RunFlywheel(launcher, 1));
        xbox.y().whileTrue(new RunFeed(launcher, 1).raceWith(new WaitCommand(1)));
        xbox.leftBumper().whileTrue(new RunLeftClimb(climb, 1));
        xbox.leftTrigger().whileTrue(new RunLeftClimb(climb, -1));
        xbox.rightBumper().whileTrue(new RunRightClimb(climb, 1));
        xbox.rightTrigger().whileTrue(new RunRightClimb(climb, -1));
        xbox.povUp().onTrue(new SetIntakeState(intake, States.OPEN));
        xbox.povDown().onTrue(new SetIntakeState(intake, States.CLOSED));
    }

    /**
     * Passes the autonomous command to the {@link Robot} class.
     *
     * @return The command to run in Autonomous mode.
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("2024 Auto");
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