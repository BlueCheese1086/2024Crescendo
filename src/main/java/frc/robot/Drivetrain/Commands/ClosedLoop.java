package frc.robot.Drivetrain.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;

public class ClosedLoop extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> zRotateSupplier;

    public ClosedLoop(Supplier<Double> xSpeedSupplier, Supplier<Double> zRotateSupplier) {
        this.drivetrain = Drivetrain.getInstance();
        this.xSpeedSupplier = xSpeedSupplier;
        this.zRotateSupplier = zRotateSupplier;
    }

    @Override
    public void execute() {
        drivetrain.closedLoop(
            new ChassisSpeeds(
                xSpeedSupplier.get() * DriveConstants.maxDriveSpeed,
                0,
                zRotateSupplier.get() * DriveConstants.maxTurnSpeed
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.closedLoop(new ChassisSpeeds());
    }
}
