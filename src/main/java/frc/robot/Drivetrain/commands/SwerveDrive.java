package frc.robot.Drivetrain.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;

public class SwerveDrive extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> xTraverseSupplier;
    private Supplier<Double> zTraverseSupplier;
    private Supplier<Double> zRotateSupplier;

    /** 
     * Creates a new SwerveDrive command. 
     * 
     * @param drivetrain A representation of the {@link Drivetrain} class that this subsystem manipulates.
     * @param xSpeedSupplier A supplier that will give the command the percent speed that the robot should move forward at.
     * @param zRotateSupplier A supplier that will give the command the percent speed that the robot should turn at.
     */
    public SwerveDrive(Drivetrain drivetrain, Supplier<Double> xTraverseSupplier, Supplier<Double> zTraverseSupplier, Supplier<Double> zRotateSupplier) {
        this.drivetrain = drivetrain;
        this.xTraverseSupplier = xTraverseSupplier;
        this.zTraverseSupplier = zTraverseSupplier;
        this.zRotateSupplier = zRotateSupplier;

        addRequirements(drivetrain);
    }

    /** This function is called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        // This is a very helpful class that will take my axes and automatically converts them into
        // the speed and angle that the motors need to be at.  It also takes in the current angle
        // of the robot to adjust the turn angle as needed
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xTraverseSupplier.get() * DriveConstants.maxDriveSpeed,
            zTraverseSupplier.get() * DriveConstants.maxDriveSpeed,
            zRotateSupplier.get() * DriveConstants.maxTurnSpeed,
            drivetrain.getAngle()
        );

        drivetrain.drive(speeds);
    }

    /** This function is called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        // Stops the motors
        drivetrain.drive(new ChassisSpeeds());
    }
}