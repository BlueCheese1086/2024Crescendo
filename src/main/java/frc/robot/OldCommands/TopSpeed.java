package frc.robot.OldCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.*;

public class TopSpeed extends Command{
 
    private final shooterSubystem m_subsystem;
    double change;

    public TopSpeed(shooterSubystem shooterSubystem, double change) {
    this.change = change;
    m_subsystem = shooterSubystem;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubystem);
    }
    
    public void initialize() {
        ShooterConstants.UPPER_POWER += change;
        SmartDashboard.putNumber("Shooter power", ShooterConstants.UPPER_POWER);
    }
}
