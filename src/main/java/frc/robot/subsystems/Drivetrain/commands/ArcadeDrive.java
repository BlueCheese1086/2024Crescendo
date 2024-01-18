package frc.robot.subsystems.Drivetrain.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain.Drivetrain;

public class ArcadeDrive extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> zRotateSupplier;

    /** Creates a new ArcadeDrive command. */
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
        drivetrain.arcadeDrive(xSpeedSupplier.get(), zRotateSupplier.get());
    }

    /** This function returns true when the command should end.  It runs at the same time as the {@linkplain #execute() execute()} function */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** This function is called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {}
}