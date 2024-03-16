// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import frc.robot.Drivetrain.Commands.SwerveDrive;
import frc.robot.Drivetrain.Drivetrain;

public class RobotContainer {
    // Creating the subsystems
    Drivetrain drivetrain = new Drivetrain();

    // Creating the controllers
    CommandXboxController driveController = new CommandXboxController(0);
    CommandXboxController operatorController = new CommandXboxController(1);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public Command getTeleopCommand() {
        return new SwerveDrive(drivetrain, 
                               () -> MathUtil.applyDeadband(driveController.getLeftX(), 0.2),
                               () -> MathUtil.applyDeadband(driveController.getLeftY(), 0.2),
                               () -> MathUtil.applyDeadband(driveController.getRightX(), 0.2)
        );
    }
}
