package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;

public class Align extends Command{
 
    private final driveSubsystem driveSubsystem;

    public Align(driveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double yaw = cameraSubsystem.getCameraYaw();
        double pitch = cameraSubsystem.getCameraPitch();
        SmartDashboard.putNumber("Yaw", yaw);
        SmartDashboard.putNumber("Pitch", pitch);
        if(Math.abs(yaw) > DriveConstants.YAW_DEADBAND){
            driveSubsystem.driveAlignYaw(yaw);
        }
        else if(pitch > -15.0){
            driveSubsystem.driveChassis(1.0, 1.0);
        }
        else{
            driveSubsystem.driveChassis(0.0, 0.0);
        }
    }
}
