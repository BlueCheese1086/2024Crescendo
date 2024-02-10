package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.  The TimedRobot class will poll the CommandScheduler every 20 ms by default.
 * This poll speed can be changed, but it is not recommended to change this.
 */
public class Robot extends TimedRobot {
    // These are the commands for each mode
    private Command autonomousCommand;
    private Command teleopCommand;

    /** This function is called once when the robot is first started up. */
    @Override
    public void robotInit() {
        System.out.println("Robot Initializing");

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        RobotContainer robotContainer = new RobotContainer();

        autonomousCommand = robotContainer.getAutonomousCommand();
        teleopCommand = robotContainer.getTeleopCommand();
    }

    /** This function is called periodically no matter the mode. */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once when the robot enters Autonomous mode. */
    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Robot Mode", "Autonomous");
        autonomousCommand.schedule();
    }

    /** This function is called periodically during Autonomous mode. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when the robot exits Autonomous mode. */
    @Override
    public void autonomousExit() {
        autonomousCommand.cancel();
    }

    /** This function is called once when the robot enters Teleop mode. */
    @Override
    public void teleopInit() {
        SmartDashboard.putString("Robot Mode", "Teleop");
        teleopCommand.schedule();
    }

    /** This function is called periodically during Teleop mode. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when the robot exits Teleop mode. */
    @Override
    public void teleopExit() {
        teleopCommand.cancel();
    }

    /** This function is called once when the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        SmartDashboard.putString("Robot Mode", "Disabled");
    }

    /** This function is called periodically during Disabled mode. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once when the robot exits Disabled mode. */
    @Override
    public void disabledExit() {}

    /** This function is called once when the robot enters Test mode. */
    @Override
    public void testInit() {
        SmartDashboard.putString("Robot Mode", "Testing");
    }

    /** This function is called periodically during Test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot exits Test mode. */
    @Override
    public void testExit() {}

    /** This function is called once when the robot enters Simulation mode. */
    @Override
    public void simulationInit() {
        SmartDashboard.putString("Robot Mode", "Simulation");
    }

    /** This function is called periodically during Simulation mode. */
    @Override
    public void simulationPeriodic() {}
}