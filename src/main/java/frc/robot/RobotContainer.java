// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Util.ControllableConfiguration;
import Util.IntializedSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Climb.Climb;
import frc.robot.Climb.Commands.SetClimbPos;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.Commands.DefaultDrive;
import frc.robot.Intake.Intake;
import frc.robot.Intake.Commands.RunRollers;
import frc.robot.Intake.Commands.SetAngle;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Commands.RunShooter;

public class RobotContainer {

	Drivetrain drivetrain = new Drivetrain();
	Climb climb = new Climb();
	Shooter shooter = new Shooter();
	Intake intake = new Intake();

	CommandXboxController primary, secondary;
	ControllableConfiguration climbDown = new ControllableConfiguration("Climb", "ClimbIsDown", false);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings


		primary = new CommandXboxController(0);
		secondary = new CommandXboxController(1);

		for (SubsystemBase s : new SubsystemBase[]{drivetrain, climb, shooter}) {
			((IntializedSubsystem) s).initialize();
		}

		drivetrain.setDefaultCommand(
			new DefaultDrive(
				() -> primary.getLeftY(), 
				() -> primary.getLeftX(), 
				() -> primary.getRightX(), 
				() -> primary.rightBumper().getAsBoolean(),
				drivetrain)
		);

		configureBindings();
	}

	private void configureBindings() {

		primary.y().onTrue(new InstantCommand(() -> {
			drivetrain.initPigeon();
		}));

		secondary.a().whileTrue(new SetAngle(0.5, intake).alongWith(new RunRollers(true, intake)));
		secondary.b().whileTrue(new SetAngle(0.5, intake).alongWith(new RunRollers(false, intake)));


		secondary.y().toggleOnTrue(new RunShooter(5500.0, 0.0, shooter));
		secondary.x().whileTrue(new RunShooter(5500.0, 15000, shooter));
		secondary.pov(90).whileTrue(new RunShooter(-5500, -5500, shooter));

		secondary.rightTrigger(0.1).whileTrue(new SetClimbPos(-1, 0, climb));
		secondary.leftTrigger(0.1).whileTrue(new SetClimbPos(0, -1, climb));
		secondary.rightBumper().whileTrue(new SetClimbPos(-1, 1, climb));
		secondary.leftBumper().whileTrue(new SetClimbPos(1, -1, climb));

	}

	public void checkClimb() {
		if ((Boolean) climbDown.getValue()) {
			climb.initialize();
		} else {
			climb.setEncToTop();
		}
	}

	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Commands.print("Test");
	}
}
