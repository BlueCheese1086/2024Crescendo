package frc.robot.Drivetrain.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;

public class DefaultDrive extends Command {

    private final DoubleSupplier x_trans;
    private final DoubleSupplier y_trans;
    private final DoubleSupplier z_rot;

    private final Drivetrain drivetrain;

    public DefaultDrive(DoubleSupplier x_trans, DoubleSupplier y_trans, DoubleSupplier z_rot, Drivetrain drivetrain) {
        this.x_trans = x_trans;
        this.y_trans = y_trans;
        this.z_rot = z_rot;

        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public void initialize() {}

    public void execute() {
        drivetrain.drive(ChassisSpeeds.fromRobotRelativeSpeeds(
            x_trans.getAsDouble() * DriveConstants.maxWheelVelocity, 
            y_trans.getAsDouble() * DriveConstants.maxWheelVelocity,
            z_rot.getAsDouble() * DriveConstants.maxRotationalVelocity, 
            drivetrain.getYaw()));
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interr) {
        drivetrain.stop();
    }
    
}