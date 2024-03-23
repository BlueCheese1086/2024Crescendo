package frc.robot.Drivetrain.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Constants.DriveConstants;

public class SwerveDrive extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> xTransSupplier;
    private Supplier<Double> yTransSupplier;
    private Supplier<Double> xRotSupplier;

    public SwerveDrive(Supplier<Double> xTransSupplier, Supplier<Double> yTransSupplier, Supplier<Double> xRotSupplier) {
        this.drivetrain = Drivetrain.getInstance();
        this.xTransSupplier = xTransSupplier;
        this.yTransSupplier = yTransSupplier;
        this.xRotSupplier = xRotSupplier;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            xTransSupplier.get() * DriveConstants.maxDriveSpeed, 
            yTransSupplier.get() * DriveConstants.maxDriveSpeed, 
            xRotSupplier.get() * DriveConstants.maxTurnSpeed, 
            drivetrain.getAngle()
        );

        drivetrain.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
