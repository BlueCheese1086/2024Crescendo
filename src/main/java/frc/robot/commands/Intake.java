// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Launcher;

import edu.wpi.first.wpilibj2.command.Command;

/** 
 * Intake command. Runs upper and lower wheels in reverse while command is running
 */
public class Intake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Launcher launcher;

  /**
   * Creates a new Intake command.
   *
   * @param launcher Launcher subsystem.
   */
  public Intake(Launcher launcher) {
    this.launcher = launcher;
    // Declare launcher subsystem dependency.
    addRequirements(launcher);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command is run until closed by container
    return false;
  }
}
