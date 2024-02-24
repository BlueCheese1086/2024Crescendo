package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Drivetrain.commands.ArcadeDrive;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Launcher.commands.RunFeed;
import frc.robot.Launcher.commands.RunFlywheel;
import frc.robot.Launcher.Launcher;

public class RobotContainer {
    // Defining the robot's subsystems
    public final Drivetrain drivetrain = new Drivetrain();
    private final Launcher launcher = new Launcher();

    // Creating instances of the xbox remotes used for driving the robot.
    public final CommandXboxController xbox = new CommandXboxController(0);

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        Pathfinding.setPathfinder(new LocalADStar());

        NamedCommands.registerCommand("RunFeed", new RunFeed(launcher, 1));
        NamedCommands.registerCommand("RunFlywheel", new RunFlywheel(launcher, 1));

        // Configuring the trigger bindings
        configureBindings();
    }

    /** Creates button bindings for the necessary functions. */
    private void configureBindings() {
        // Assigning operations to each button.
        // "A" runs the launcher in reverse to collect notes.
        // "B" resets the encoders and odometry.
        // "Left Bumper" runs the flywheel on the launcher.
        // "Right Bumper" runs the feed wheel on the launcher.
        xbox.a().whileTrue(new RunFlywheel(launcher, -1));
        xbox.leftBumper().whileTrue(new RunFlywheel(launcher, 1));
        xbox.rightBumper().whileTrue(new RunFeed(launcher, 1));
    }

    /**
     * Passes the autonomous command to the {@link Robot} class.
     *
     * @return The command to run in Autonomous mode.
     */
    public Command getAutonomousCommand() {
        Pose2d targetPose = new Pose2d(3, 6, new Rotation2d());
        PathConstraints constraints = new PathConstraints(0.5, 1, 10, 3);
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    /**
     * Passes the teleop command to the {@link Robot} class.
     *
     * @return The command to run in Teleop mode.
     */
    public Command getTeleopCommand() {
        return new ArcadeDrive(drivetrain, () -> xbox.getLeftY(), () -> xbox.getRightX());
    }
}