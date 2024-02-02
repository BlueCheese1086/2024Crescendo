package frc.robot.Drivetrain.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Drivetrain.Drivetrain;

public class DriveTime extends Command {
    private Drivetrain drivetrain;
    private double xSpeed;
    private double zRotate;
    private double endTime;

    /**
     * Creates a new DriveTime command.
     * 
     * @param drivetrain A representation of the {@link Drivetrain} class that this subsystem manipulates.
     * @param xSpeed The percent speed that the robot should move forward at.
     * @param zRotate The percent speed that the robot should turn at.
     * @param seconds The amount of time in seconds that the command will run.
     */
    public DriveTime(Drivetrain drivetrain, double xSpeed, double zRotate, double seconds) {
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.zRotate = zRotate;
        this.endTime = System.currentTimeMillis() + (seconds * 1000);

        addRequirements(drivetrain);
    }
    
    /** This function is called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** This function is called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        drivetrain.arcadeDrive(xSpeed, zRotate);
    }

    /** This function returns true when the command should end.  It runs at the same time as the {@linkplain #execute() execute()} function */
    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > endTime;
    }

    /** This function is called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }
}