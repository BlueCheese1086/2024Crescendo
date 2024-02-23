package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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

    // Creating instances of the xbox remotes used for driving the robot.
    private final CommandJoystick joystick = new CommandJoystick(0);

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        // Not Implementing PathPlanner until the robot can run normally.
        // Configuring the NamedCommands for PathPlanner
        // NamedCommands.registerCommand("ShootNote", new RunLauncher(launcher, 1, 3, 0.5));
        // NamedCommands.registerCommand("RunIntake", new RunIntake(intake, 1));
        // NamedCommands.registerCommand("OpenIntake", new SetIntakeState(intake, States.OPEN));
        // NamedCommands.registerCommand("CloseIntake", new SetIntakeState(intake, States.CLOSED));

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
        joystick.button(1).whileTrue(new RunIntake(intake, 0.25)); // A
        joystick.button(2).whileTrue(new RunIntake(intake, -0.25)); // B
        joystick.button(3).toggleOnTrue(new RunFlywheel(launcher, 1)); // X
        joystick.button(4).whileTrue(new RunFeed(launcher, 1).raceWith(new WaitCommand(1))); // Y
        joystick.button(5).whileTrue(new RunLeftClimb(climb, 0.25)); // Left Bumper
        joystick.axisGreaterThan(2, 0.5).whileTrue(new RunLeftClimb(climb, -0.25)); // Left Trigger
        joystick.button(6).whileTrue(new RunRightClimb(climb, 0.25)); // Right Bumper
        joystick.axisGreaterThan(3, 0.5).whileTrue(new RunRightClimb(climb, -0.25)); // Right Trigger
        joystick.povUp().whileTrue(new SetIntakeState(intake, States.OPEN)); // POV Down
        joystick.povDown().whileTrue(new SetIntakeState(intake, States.CLOSED)); // POV Up
    }

    /**
     * Passes the autonomous command to the {@link Robot} class.
     *
     * @return The command to run in Autonomous mode.
     */
    public Command getAutonomousCommand() {
        return null;//new PathPlannerAuto("2024 Auto");
    }

    /**
     * Passes the teleop command to the {@link Robot} class.
     *
     * @return The command to run in Teleop mode.
     */
    public Command getTeleopCommand() {
        return new SwerveDrive(drivetrain,
            () -> MathUtil.applyDeadband(joystick.getRawAxis(0), DriveConstants.deadband), // X Traverse
            () -> MathUtil.applyDeadband(joystick.getRawAxis(1), DriveConstants.deadband), // Y Traverse
            () -> MathUtil.applyDeadband(joystick.getRawAxis(4), DriveConstants.deadband)  // Z Rotate
        );
    }
}