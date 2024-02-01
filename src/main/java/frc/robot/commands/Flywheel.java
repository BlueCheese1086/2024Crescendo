// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LauncherConstants.*;

import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.Command;

/** 
 * Flywheel command. Runs upper wheel while command is running.
 */
public class Flywheel extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Launcher m_launcher;

  /**
   * Creates a new Flywheel command.
   *
   * @param launcher Launcher subsystem.
   */
  public Flywheel(Launcher launcher) {
    m_launcher = launcher;
    // Declare launcher subsystem dependency.
    addRequirements(launcher);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.setUpper(FlywheelSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the wheel; control systems are unnecessary.
    m_launcher.stopUpper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command is run until closed by container
    return false;
  }
}
