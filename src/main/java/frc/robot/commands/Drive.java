package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveSubsystem;

public class Drive extends Command  {
    private final DoubleSupplier rightSpeedF;
    private final DoubleSupplier leftSpeedF;
    private final DoubleSupplier rightSpeedB;
    private final DoubleSupplier leftSpeedB;
    private final driveSubsystem m_subsystem;

    public Drive(driveSubsystem subsystem, DoubleSupplier leftSpeedF, DoubleSupplier rightSpeedF, DoubleSupplier leftSpeedB, DoubleSupplier rightSpeedB) {
        m_subsystem = subsystem;
        this.rightSpeedF = rightSpeedF;
        this.leftSpeedF = leftSpeedF;
        this.rightSpeedB = rightSpeedB;
        this.leftSpeedB = leftSpeedB;
      
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }
      
      public void execute() {
        m_subsystem.set(leftSpeedF.getAsDouble(), rightSpeedF.getAsDouble(), leftSpeedB.getAsDouble(), rightSpeedB.getAsDouble());
        SmartDashboard.putNumber("Left Speed:", leftSpeedF.getAsDouble());
        SmartDashboard.putNumber("Right Speed:", rightSpeedF.getAsDouble());
      }
}
