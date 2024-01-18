package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class Shoot extends CommandBase{
 
    private final shooterSubystem m_subsystem;
    private final BooleanSupplier shootDo;
    //private final DoubleSupplier intakeSpeed;

    XboxController joy = new XboxController(0);

    public Shoot(shooterSubystem shooterSubystem, BooleanSupplier shootDo) {
        this.shootDo = shootDo;

        m_subsystem = shooterSubystem;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubystem);
    }
    
    public void execute() {;
        shooterSubystem.shoot(shootDo.getAsBoolean());
    }
}
