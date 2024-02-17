package frc.robot.Drivetrain.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;

public class ArcadeDrive extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> zRotateSupplier;

    /**
     * Creates a new ArcadeDrive command.
     * 
     * @param drivetrain      A representation of the {@link Drivetrain} class that this subsystem manipulates.
     * @param xSpeedSupplier  A supplier that will give the command the percent speed that the robot should move forward at.
     * @param zRotateSupplier A supplier that will give the command the percent speed that the robot should turn at.
     */
    public ArcadeDrive(Drivetrain drivetrain, Supplier<Double> xSpeedSupplier, Supplier<Double> zRotateSupplier) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.zRotateSupplier = zRotateSupplier;

        addRequirements(drivetrain);
    }

    /** This function is called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** This function is called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        // Getting the values from the suppliers.
        double xSpeed = MathUtil.applyDeadband(xSpeedSupplier.get(), DriveConstants.deadband);
        double zRotate = MathUtil.applyDeadband(zRotateSupplier.get(), DriveConstants.deadband);

        // Converting the inputs to left and right speeds.
        double leftSpeed = xSpeed - zRotate;
        double rightSpeed = xSpeed + zRotate;

        // Saturating the inputs.
        double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotate));
        double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotate));
        if (greaterInput != 0) {
            double saturatedInput = (greaterInput + lesserInput) / greaterInput;
            leftSpeed /= saturatedInput;
            rightSpeed /= saturatedInput;
        }

        // Converting the left and right speeds to ChassisSpeeds.
        ChassisSpeeds speeds = drivetrain.kinematics
                .toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));

        // Giving the ChassisSpeeds to the drive function.
        drivetrain.drive(speeds);
    }

    /**
     * This function returns true when the command should end.
     * It runs at the same time as the {@linkplain #execute() execute()} function.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** This function is called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {}
}