package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class Intake extends Command{
 
    private final shooterSubystem m_subsystem;
    private final boolean intakeDo;

    public Intake(shooterSubystem shooterSubystem, boolean intakeDo) {
        this.intakeDo = intakeDo;
        m_subsystem = shooterSubystem;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubystem);
    }
    
    public void execute() {
        m_subsystem.intake(intakeDo);
    }
}
