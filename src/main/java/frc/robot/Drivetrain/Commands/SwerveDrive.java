package frc.robot.Drivetrain.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveDrive extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> ySpeedSupplier;
    private Supplier<Double> zRotatSupplier;

    public SwerveDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> zRotatSupplier) {
        this.drivetrain = Drivetrain.getInstance();
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.zRotatSupplier = zRotatSupplier;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xSpeed = xSpeedSupplier.get();
        xSpeed = MathUtil.applyDeadband(xSpeed, ControllerConstants.deadband);
        if (xSpeed > 0) xSpeed -= ControllerConstants.deadband;
        if (xSpeed < 0) xSpeed += ControllerConstants.deadband;
        xSpeed *= DriveConstants.maxDriveSpeed;

        double ySpeed = ySpeedSupplier.get();
        ySpeed = MathUtil.applyDeadband(ySpeed, ControllerConstants.deadband);
        if (ySpeed > 0) ySpeed -= ControllerConstants.deadband;
        if (ySpeed < 0) ySpeed += ControllerConstants.deadband;
        ySpeed *= DriveConstants.maxDriveSpeed;

        double zRotat = zRotatSupplier.get();
        zRotat = MathUtil.applyDeadband(zRotat, ControllerConstants.deadband);
        if (zRotat > 0) zRotat -= ControllerConstants.deadband;
        if (zRotat < 0) zRotat += ControllerConstants.deadband;
        zRotat *= DriveConstants.maxTurnSpeed;


        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zRotat, drivetrain.getAngle());

        if (xSpeed == 0 && ySpeed == 0 && zRotat == 0) {
            drivetrain.makeX();
        } else {
            drivetrain.drive(speeds);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
