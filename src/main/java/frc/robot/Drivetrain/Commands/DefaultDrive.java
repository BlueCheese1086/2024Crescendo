package frc.robot.Drivetrain.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;

public class DefaultDrive extends Command {

    private final DoubleSupplier x_trans;
    private final DoubleSupplier y_trans;
    private final DoubleSupplier z_rot;

    private final BooleanSupplier marioKart;

    private final Drivetrain drivetrain;

    public DefaultDrive(DoubleSupplier x_trans, DoubleSupplier y_trans, DoubleSupplier z_rot, BooleanSupplier marioKart, Drivetrain drivetrain) {
        this.x_trans = x_trans;
        this.y_trans = y_trans;
        this.z_rot = z_rot;

        this.marioKart = marioKart;

        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public void initialize() {}

    public void execute() {
        ChassisSpeeds speeds = drivetrain.getSpeeds();

        drivetrain.drive(ChassisSpeeds.fromRobotRelativeSpeeds(
            x_trans.getAsDouble() * DriveConstants.maxWheelVelocity + (marioKart.getAsBoolean() ? Math.signum(x_trans.getAsDouble()) * ((DriveConstants.maxWheelVelocity-speeds.vxMetersPerSecond)/DriveConstants.maxWheelVelocity * DriveConstants.maxWheelVelocity) : 0.0), 
            y_trans.getAsDouble() * DriveConstants.maxWheelVelocity + (marioKart.getAsBoolean() ? Math.signum(y_trans.getAsDouble()) * ((DriveConstants.maxWheelVelocity-speeds.vxMetersPerSecond)/DriveConstants.maxWheelVelocity * DriveConstants.maxWheelVelocity) : 0.0),
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