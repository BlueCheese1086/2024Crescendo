// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Drivetrain.Commands.*;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Commands.*;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Commands.*;
import frc.robot.Shooter.Shooter;
import frc.robot.Tower.Commands.*;
import frc.robot.Tower.Tower;
import frc.robot.Vision.Vision;

public class RobotContainer {
    // Creating the subsystems
    Vision vision = new Vision();
    Drivetrain drivetrain = new Drivetrain();
    Shooter shooter = new Shooter();
    Intake intake = new Intake();
    Tower tower = new Tower();

    // Creating the controllers
    CommandXboxController driveController = new CommandXboxController(0);
    CommandXboxController operatorController = new CommandXboxController(1);

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
            POV Up: Stowing the intake.
            POV Down: Extending the intake.
        */
        operatorController.a().whileTrue(new SetFeedSpeed(1));
        operatorController.b().whileTrue(new SetIntakeSpeed(1));
        operatorController.x().whileTrue(new SetLauncherSpeed(-1));
        operatorController.y().toggleOnTrue(new SetLauncherSpeed(1));
        operatorController.leftBumper().whileTrue(new SetShooterAngle(shooter.getAngle(), 1));
        operatorController.leftTrigger().whileTrue(new SetShooterAngle(shooter.getAngle(), -1));
        operatorController.rightBumper().whileTrue(new SetTowerSpeed(1));
        operatorController.rightTrigger().whileTrue(new SetTowerSpeed(-1));
        operatorController.back().whileTrue(new SetIntakeSpeed(-1));
        operatorController.povUp().onTrue(new SetIntakeAngle(Rotation2d.fromDegrees(90)));
        operatorController.povDown().onTrue(new SetIntakeAngle(new Rotation2d()));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("v3-auto");
    }

    public Command getTeleopCommand() {
        return new SwerveDrive(drivetrain, 
            () -> MathUtil.applyDeadband(driveController.getLeftX(), 0.2),
            () -> MathUtil.applyDeadband(driveController.getLeftY(), 0.2),
            () -> MathUtil.applyDeadband(driveController.getRightX(), 0.2)
        );
    }
}
