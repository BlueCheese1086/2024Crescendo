package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class Intake extends Command{
 
    private final shooterSubystem m_subsystem;

    public Intake(shooterSubystem shooterSubystem) {
        m_subsystem = shooterSubystem;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubystem);
    }
    
    public void execute() {
        m_subsystem.intake(true);
    }

    public void end(boolean interrupted){
        m_subsystem.intake(false);
    }
}
