package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.driveSubsystem;

public class Drive extends Command  {
    //private final DoubleSupplier rotateSpeed;
    //private final DoubleSupplier driveSpeed;
    private final Supplier<ChassisSpeeds> speed;
    private final driveSubsystem m_subsystem;
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));

    public Drive(driveSubsystem subsystem, Supplier<ChassisSpeeds> speed) {
        this.speed = speed;
        m_subsystem = subsystem;
        //this.rotateSpeed = rotateSpeed;
        //this.driveSpeed = driveSpeed;
      
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }
      
      public void execute() {
        SmartDashboard.putNumber("Left Y", speed.get().vxMetersPerSecond);
        SmartDashboard.putNumber("Left X", speed.get().omegaRadiansPerSecond);
        var wheelSpeeds = kinematics.toWheelSpeeds(speed.get());
        m_subsystem.driveChassis(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
      }
}
