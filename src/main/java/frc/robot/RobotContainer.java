// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.Commands.ArcadeDrive;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Commands.RunFeed;
import frc.robot.Shooter.Commands.RunShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Shooter shooter = new Shooter();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        operator.a().whileTrue(new RunFeed(shooter, 1));
        operator.b().whileTrue(new RunShooter(shooter, -1)).whileTrue(new RunFeed(shooter, -1));
        operator.y().toggleOnTrue(new RunShooter(shooter, 1));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        var commands = Commands.sequence(
            Commands.deadline(
                new WaitCommand(2),
                Commands.parallel(
                    Commands.deadline(
                        new WaitCommand(0.2),
                        new RunFeed(shooter, -1)
                    ),
                    new RunShooter(shooter, 1),
                    Commands.sequence(
                        new WaitCommand(1),
                        new RunFeed(shooter, 1)
                    )
                )
            ),
            Commands.deadline(
                new WaitCommand(2),
                new ArcadeDrive(m_drivetrain, () -> 0.5, () -> 0.0)
            )
        );

        return commands;
    }

    public Command getTeleopCommand() {
        return new ArcadeDrive(m_drivetrain, ()-> driver.getLeftY(), ()-> driver.getRightX());
    }
}