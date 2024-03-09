// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LEDManager;
import frc.robot.commands.LEDActivate;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import org.photonvision.estimation.CameraTargetRelation;

import com.ctre.phoenix6.sim.ChassisReference;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final driveSubsystem m_DriveSubsystem = new driveSubsystem();
  private final shooterSubystem m_ShooterSubsystem = new shooterSubystem();
  private final cameraSubsystem m_CameraSubsystem = new cameraSubsystem();
  
  ChassisSpeeds speeds = new ChassisSpeeds();

  CommandXboxController xbox = new CommandXboxController(0);
  XboxController joy = xbox.getHID();

  LEDManager leds = new LEDManager(false);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Shoot", new Shoot(m_ShooterSubsystem));
    NamedCommands.registerCommand("Intake", new Intake(m_ShooterSubsystem));
    NamedCommands.registerCommand("Align", new Align(m_DriveSubsystem));

    // Configure the trigger bindings
    configureBindings();

    m_DriveSubsystem.setDefaultCommand(
      new Drive(m_DriveSubsystem, () -> new ChassisSpeeds(
        MathUtil.applyDeadband(-joy.getLeftY(), DriveConstants.DEADBAND)*DriveConstants.MAX_DRIVE_SPEED, 
        0, 
        MathUtil.applyDeadband(-joy.getRightX(), DriveConstants.DEADBAND)*DriveConstants.MAX_TURN_SPEED
      ))
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}
   */
  private void configureBindings() {
    //Whack controller:
    /*xbox.button(4).whileTrue(new AlignYaw(m_DriveSubsystem, true));
    xbox.button(3).onTrue(new Shoot(m_ShooterSubsystem));
    xbox.button(2).whileTrue(new Intake(m_ShooterSubsystem));
    xbox.button(1).onTrue(new AutoShoot(m_DriveSubsystem, m_ShooterSubsystem));
    xbox.pov(0).whileTrue(new TopSpeed(m_ShooterSubsystem, 0.01));
    xbox.pov(180).whileTrue(new TopSpeed(m_ShooterSubsystem, -0.01));*/

    //Normal controller:
    //xbox.x().whileTrue(new AlignYaw(m_DriveSubsystem, true));
    //xbox.a().onTrue(new Shoot(m_ShooterSubsystem));
    //xbox.b().whileTrue(new Intake(m_ShooterSubsystem));
    xbox.x().whileTrue(new Align(m_DriveSubsystem));
    xbox.y().whileTrue(new AlignDrive(m_DriveSubsystem, 
    () -> new ChassisSpeeds(
        MathUtil.applyDeadband(-joy.getLeftY(), DriveConstants.DEADBAND)*DriveConstants.MAX_DRIVE_SPEED, 
        0, 
        MathUtil.applyDeadband(-joy.getLeftX(), DriveConstants.DEADBAND)*DriveConstants.MAX_TURN_SPEED)));
    //xbox.y().onTrue(new AutoShoot(m_DriveSubsystem, m_ShooterSubsystem));
    xbox.leftBumper().onTrue(new InstantCommand(() -> m_DriveSubsystem.resetOdometry()));
    xbox.start().toggleOnTrue(new LEDActivate(leds));
  }

    public Command getAutonomousCommand() {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("1 meter");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);

        //Auto alternative:
        //return new PathPlannerAuto("1 note");
    }
}
