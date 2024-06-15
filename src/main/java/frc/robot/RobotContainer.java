// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Drivetrain.Commands.*;
import frc.robot.Climb.Climb;
import frc.robot.Climb.Commands.*;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Commands.*;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Commands.*;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Pivot;

public class RobotContainer {
    // Creating the subsystems
    Drivetrain drivetrain = Drivetrain.getInstance();
    Shooter shooter = Shooter.getInstance();
    Pivot pivot = Pivot.getInstance();
    Intake intake = Intake.getInstance();
    Climb tower = Climb.getInstance();

    // Creating the controllers
    CommandXboxController joystick = new CommandXboxController(1);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        /*  All operator controls. -- See Controls.png for visual reference.
            A: Runs the feed
            B: Runs the intake
            X: Runs the launcher in reverse
            Y: Toggles the launcher
            Left Bumper: Moves the shooter up.
            Left Trigger: Moves the shooter down.
            Right Bumper: Moves the tower up.
            Right Trigger: Moves the tower down.
            Back: Running the intake in reverse.
            D-Pad Up: Stowing the intake.
            D-Pad Down: Extending the intake.
            Left Joystick: Move up/down/left/right respectively
            Right Joystick: Turn left/right respectively
        */
        joystick.a().whileTrue(new SetFeedSpeed(1));
        joystick.b().whileTrue(new SetIntakeSpeed(1));
        joystick.x().whileTrue(new SetLauncherSpeed(-1));
        joystick.y().toggleOnTrue(new SetLauncherSpeed(1));
        joystick.leftBumper().whileTrue(new SetShooterAngle(pivot.getAngle(), 1));
        joystick.leftTrigger().whileTrue(new SetShooterAngle(pivot.getAngle(), -1));
        joystick.rightBumper().whileTrue(new SetClimbSpeed(1));
        joystick.rightTrigger().whileTrue(new SetClimbSpeed(-1));
        joystick.back().whileTrue(new SetIntakeSpeed(-1));
        joystick.povUp().onTrue(new SetIntakeAngle(Intake.States.CLOSED));
        joystick.povDown().onTrue(new SetIntakeAngle(Intake.States.OPEN));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("V3-Auto");
    }

    public Command getTeleopCommand() {
        return new SwerveDrive(
            () -> MathUtil.applyDeadband(joystick.getLeftX(), 0.2),
            () -> MathUtil.applyDeadband(joystick.getLeftY(), 0.2),
            () -> MathUtil.applyDeadband(joystick.getRightX(), 0.2)
        );
    }
}
