// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/** 
 * Drive command. Passes arcade drive inputs to the {@link Drivetrain drivetrain} subsystem.
 */
public class Drive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;
  private final Supplier<ChassisSpeeds> speedSupplier;

  /**
   * Creates a new Drive command.
   *
   * @param drivetrain Drivetrain subsystem.
   */
  public Drive(Drivetrain drivetrain, Supplier<ChassisSpeeds> speedSupplier) {
    this.drivetrain = drivetrain;
    this.speedSupplier = speedSupplier;
    // Declare drivetrain subsystem dependency.
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setSpeeds(speedSupplier.get());  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command is run until closed by container
    return false;
  }
}
