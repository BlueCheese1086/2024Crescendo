package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class Shoot extends Command{
 
    private final shooterSubystem m_subsystem;

    private long start;

    public Shoot(shooterSubystem shooterSubystem) {
        m_subsystem = shooterSubystem;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubystem);
    }

    public void initialize(){
        start = System.currentTimeMillis();
    }
    
    public void execute() {
        if(start + 500 > System.currentTimeMillis()){
            SmartDashboard.putString("Stage:", "Stage 1");
            m_subsystem.shootUpper(true);
        }
        else if (start+500 < System.currentTimeMillis() && start + 1000 > System.currentTimeMillis()){
            SmartDashboard.putString("Stage:", "Stage 2");
            m_subsystem.shootUpper(true);
            m_subsystem.shootLower(true);
        }
    }

    public boolean isFinished(){
        return start + 1000 <= System.currentTimeMillis();
    }

    public void end(boolean interrupted){
        SmartDashboard.putString("Stage:", "Stage 3");
        m_subsystem.shootLower(false);
        m_subsystem.shootUpper(false);
    }
}
