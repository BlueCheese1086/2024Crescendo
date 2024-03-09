// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

	CommandXboxController primary, secondary;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings

		primary = new CommandXboxController(0);
		secondary = new CommandXboxController(1);
		// secondary = primary;

		configureBindings();
	}

	private void configureBindings() {

	}

	public void setGyroAngle() {
		// Gyro.getInstance().setAngle(drivetrain.getPose().getRotation().getDegrees());
	}

	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return new PrintCommand("Hello WOrld");
	}

}
