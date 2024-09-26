    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    package frc.robot;
    
    import edu.wpi.first.wpilibj.TimedRobot;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;

    public class Robot extends TimedRobot {
    private Command autoCommand;
    private Command teleopCommand;

    @Override
    public void robotInit() {
        // Creating the RobotContainer
        RobotContainer robotContainer = new RobotContainer();

        // Getting the auto and telop commands from RobotContainer.
        autoCommand = robotContainer.getAutonomousCommand();
        teleopCommand = robotContainer.getTeleopCommand();

        // Assigning default values to the commands in case they are null.
        if (autoCommand == null) autoCommand = new PrintCommand("No auto lol");
        if (teleopCommand == null) teleopCommand = new PrintCommand("No teleop lol");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autoCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        autoCommand.cancel();
    }

    @Override
    public void teleopInit() {
        teleopCommand.schedule();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        teleopCommand.cancel();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}