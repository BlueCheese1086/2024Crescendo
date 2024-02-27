// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import Util.ControllableConfiguration;
import Util.IntializedSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Climb.Climb;
import frc.robot.Climb.Commands.SetClimbPos;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.Commands.DefaultDrive;
import frc.robot.Intake.Intake;
import frc.robot.Intake.Commands.IntakeForShooter;
import frc.robot.Intake.Commands.RunRollers;
import frc.robot.Intake.Commands.SetAngle;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Commands.RunShooter;

public class RobotContainer {

	Drivetrain drivetrain = new Drivetrain();
	Climb climb = new Climb();
	Shooter shooter = new Shooter();
	Intake intake = new Intake();

	private final SendableChooser<Command> autoChooser;

	CommandXboxController primary, secondary;
	ControllableConfiguration climbDown = new ControllableConfiguration("Climb", "ClimbIsDown", true);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings

		primary = new CommandXboxController(0);
		secondary = new CommandXboxController(1);
		// secondary = primary;

		for (SubsystemBase s : new SubsystemBase[]{drivetrain, climb, shooter}) {
			((IntializedSubsystem) s).initialize();
		}

		NamedCommands.registerCommand("Intake", new IntakeForShooter(intake));
		NamedCommands.registerCommand("Shoot", new RunShooter(5500, 15000, secondary, shooter));

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(autoChooser);


		drivetrain.setDefaultCommand(
			new DefaultDrive(
				() -> MathUtil.applyDeadband(-primary.getLeftY(), 0.1), 
				() -> MathUtil.applyDeadband(-primary.getLeftX(), 0.1), 
				() -> MathUtil.applyDeadband(-primary.getRightX(), 0.1), 
				() -> primary.rightBumper().getAsBoolean(),
				drivetrain)
		);

		intake.setDefaultCommand(new SetAngle(IntakeConstants.STOWED_ANGLE, intake));

		configureBindings();
	}

	private void configureBindings() {

		primary.leftStick().onTrue(new InstantCommand(() -> {
			drivetrain.initPigeon();
		}));

		// secondary.a().whileTrue(new SetAngle(0.5, intake).alongWith(new RunRollers(true, intake)));
		// secondary.b().whileTrue(new SetAngle(0.5, intake).alongWith(new RunRollers(false, intake)));

		secondary.a().whileTrue(new IntakeForShooter(intake));
		secondary.b().whileTrue(new RunRollers(false, intake));

		secondary.y().toggleOnTrue(new RunShooter(5500.0, 0.0, secondary, shooter));
		secondary.x().whileTrue(new RunShooter(5500.0, 5500, secondary, shooter));
		secondary.pov(90).whileTrue(new RunShooter(-5500, -5500, secondary, shooter));
		secondary.pov(270).whileTrue(new RunShooter(1, 1000, primary, shooter));

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
		return autoChooser.getSelected();
		// return new PrintCommand("Hello WOrld");
	}
}
