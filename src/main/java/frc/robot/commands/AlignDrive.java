package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;

public class AlignDrive extends Command{
 
    private final driveSubsystem driveSub;
    ChassisSpeeds alignSpeeds;
    Supplier<ChassisSpeeds> speed;
    PIDController alignPID= new PIDController(0.1,0,0);
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));

    public AlignDrive(driveSubsystem driveSubsystem, Supplier<ChassisSpeeds> speed) {
        driveSub = driveSubsystem;
        this.speed = speed;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    }


    //Full robot control
    public void execute() {
        alignSpeeds = new ChassisSpeeds(
        -alignPID.calculate(cameraSubsystem.getCameraPitch(), -15.0), 
        0, 
        alignPID.calculate(MathUtil.applyDeadband(cameraSubsystem.getCameraYaw(), DriveConstants.DEADBAND), 0));
        var alignwheelSpeeds = kinematics.toWheelSpeeds(alignSpeeds);
        driveSub.driveChassis(alignwheelSpeeds.leftMetersPerSecond, alignwheelSpeeds.rightMetersPerSecond);
    }

    //Partial robot control
    /*public void execute() {
        var wheelSpeeds = kinematics.toWheelSpeeds(speed.get());
        alignSpeeds = new ChassisSpeeds(
        -alignPID.calculate(cameraSubsystem.getCameraPitch(), -15.0), 
        0, 
        alignPID.calculate(MathUtil.applyDeadband(cameraSubsystem.getCameraYaw(), DriveConstants.DEADBAND), 0));
        var alignwheelSpeeds = kinematics.toWheelSpeeds(alignSpeeds);
        driveSub.driveChassis(
        alignwheelSpeeds.leftMetersPerSecond * cameraSubsystem.getCameraArea()/10 + wheelSpeeds.leftMetersPerSecond, 
        alignwheelSpeeds.rightMetersPerSecond * cameraSubsystem.getCameraArea()/10 + wheelSpeeds.rightMetersPerSecond);
    }*/

    public void end(boolean interrupted){

    }
}
