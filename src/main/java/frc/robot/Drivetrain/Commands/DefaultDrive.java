package frc.robot.Drivetrain.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import Util.DebugPID;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.SensorsAndFeedback.Gyro;

public class DefaultDrive extends Command {

    private final DoubleSupplier x_trans;
    private final DoubleSupplier y_trans;
    private final DoubleSupplier z_rot;

    private final BooleanSupplier alignToDS;
    private boolean prevAlign;

    private final PIDController aligningPID = new PIDController(DriveConstants.rotationAlignkP, DriveConstants.rotationAlignI, DriveConstants.rotationAlignD);
    
    private final Gyro gyro;
    private final Drivetrain drivetrain;

    public DefaultDrive(DoubleSupplier x_trans, DoubleSupplier y_trans, DoubleSupplier z_rot, BooleanSupplier alignToDS, Drivetrain drivetrain) {
        this.x_trans = x_trans;
        this.y_trans = y_trans;
        this.z_rot = z_rot;

        this.alignToDS = alignToDS;
        this.prevAlign = alignToDS.getAsBoolean();

        this.drivetrain = drivetrain;
        this.gyro = Gyro.getInstance();
        new DebugPID(aligningPID, "DrivingAutoTurning");
        addRequirements(drivetrain);
    }

    public void initialize() {}

    public void execute() {
        if (alignToDS.getAsBoolean() && !prevAlign) aligningPID.reset();

        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            x_trans.getAsDouble() * DriveConstants.maxWheelVelocity,
            y_trans.getAsDouble() * DriveConstants.maxWheelVelocity,
            z_rot.getAsDouble() * DriveConstants.maxRotationalVelocity + (alignToDS.getAsBoolean() ? aligningPID.calculate(gyro.getAngle().getDegrees(), -180.0) : 0.0), 
            gyro.getAngle()));

        prevAlign = alignToDS.getAsBoolean();
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interr) {
        drivetrain.stop();
    }
    
}