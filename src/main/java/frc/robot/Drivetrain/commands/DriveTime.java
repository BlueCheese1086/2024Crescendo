package frc.robot.Drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Drivetrain.Drivetrain;

public class DriveTime extends Command {
    private Drivetrain drivetrain;
    private double xSpeed;
    private double zSpeed;
    private Rotation2d angle;
    private double endTime;

    /**
     * Creates a new DriveTime command.
     * 
     * @param drivetrain A representation of the {@link Drivetrain} class that this subsystem manipulates.
     * @param xSpeed The percent speed that the robot should move at along the x axis.
     * @param zSpeed The percent speed that the robot should move at along the z axis.
     * @param angle The angle the robot should turn to before moving. (-1 to not change) (degrees)
     * @param seconds The amount of time that the command will run. (seconds)
     */
    public DriveTime(Drivetrain drivetrain, double xSpeed, double zSpeed, double angle, double seconds) {
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.zSpeed = zSpeed;
        this.angle = (angle == -1) ? drivetrain.getAngle() : Rotation2d.fromDegrees(angle);
        this.endTime = System.currentTimeMillis() + (seconds * 1000);

        addRequirements(drivetrain);
    }
    
    /** This function is called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** This function is called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        drivetrain.swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, zSpeed, angle.getRadians(), drivetrain.getAngle()));
    }

    /** This function returns true when the command should end.  It runs at the same time as the {@linkplain #execute() execute()} function */
    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > endTime;
    }

    /** This function is called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrain.getAngle()));
    }
}