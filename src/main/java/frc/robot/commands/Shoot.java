package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class Shoot extends Command{
 
    private final shooterSubystem m_subsystem;
    private final boolean shootDo;

    private long start;

    public Shoot(shooterSubystem shooterSubystem, boolean shootDo) {
        this.shootDo = shootDo;
        m_subsystem = shooterSubystem;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubystem);
    }

    public void initialize(){
        start = System.currentTimeMillis();
    }
    
    public void execute() {
        if(start + 500 > System.currentTimeMillis()){
            m_subsystem.shootUpper(shootDo);
        }
        else {
            m_subsystem.shootUpper(shootDo);
            m_subsystem.shootLower(shootDo);
        }
    }

    public boolean isFinished(){
        return start + 1000 <= System.currentTimeMillis();
    }
}
