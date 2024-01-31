package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveSubsystem;

public class Drive extends Command  {
    private final DoubleSupplier rotateSpeed;
    private final DoubleSupplier driveSpeed;
    private final driveSubsystem m_subsystem;

    public Drive(driveSubsystem subsystem, DoubleSupplier rotateSpeed, DoubleSupplier driveSpeed) {
        m_subsystem = subsystem;
        this.rotateSpeed = rotateSpeed;
        this.driveSpeed = driveSpeed;
      
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }
      
      public void execute() {
        m_subsystem.set(rotateSpeed.getAsDouble(), driveSpeed.getAsDouble());
      }
}
