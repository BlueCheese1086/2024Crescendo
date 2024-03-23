// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import Util.ControllableConfiguration;
import Util.Interfaces.InitializedSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Climb.Climb;
import frc.robot.Climb.Commands.SetClimbPos;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.Commands.DefaultDrive;
import frc.robot.Intake.Intake;
import frc.robot.Intake.Commands.AutoIntake;
import frc.robot.Intake.Commands.IntakeDefault;
import frc.robot.Intake.Commands.SetIntakeState;
import frc.robot.Intake.Intake.IntakeState;
import frc.robot.SensorsAndFeedback.LEDFeedback;
import frc.robot.SensorsAndFeedback.LEDFeedback.LEDMode;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Commands.AutoShoot;
import frc.robot.Shooter.Commands.RunShooter;

public class RobotContainer {

	Drivetrain drivetrain = Drivetrain.getInstance();
	Climb climb = Climb.getInstance();
	Intake intake = Intake.getInstance();
	Shooter shooter = Shooter.getInstance();
	LEDFeedback leds = new LEDFeedback();
	// Watchdog wd = new Watchdog(drivetrain, drivetrain, climb);

	private final SendableChooser<Command> autoChooser;

	CommandXboxController primary, secondary;
	ControllableConfiguration climbDown = new ControllableConfiguration("Climb", "ClimbIsDown", true);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings

		primary = new CommandXboxController(0);
		secondary = new CommandXboxController(1);
		secondary = primary;

		NamedCommands.registerCommand("Intake", new AutoIntake(intake));
		NamedCommands.registerCommand("Shoot", new AutoShoot(shooter, intake));
		NamedCommands.registerCommand("IntakeThenShoot", new AutoIntake(intake).andThen(new AutoShoot(shooter, intake)));

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(autoChooser);

		for (SubsystemBase s : new SubsystemBase[]{drivetrain, climb, intake, shooter}) {
			((InitializedSubsystem) s).initialize();
		}

		drivetrain.setDefaultCommand(
			new DefaultDrive(
				() -> MathUtil.applyDeadband(-primary.getLeftY(), 0.1), 
				() -> MathUtil.applyDeadband(-primary.getLeftX(), 0.1), 
				() -> MathUtil.applyDeadband(-primary.getRightX(), 0.1), 
				() -> primary.rightBumper().getAsBoolean(),
				drivetrain)
		);

		intake.setDefaultCommand(new IntakeDefault(intake));

		configureBindings();
	}

	private void configureBindings() {

		leds.setMode(LEDMode.Bootup);

		for (InitializedSubsystem s : new InitializedSubsystem[]{drivetrain, intake, climb, shooter}) {
			s.initialize();
		}

		primary.leftStick().onTrue(new InstantCommand(() -> {
			frc.robot.SensorsAndFeedback.Gyro.getInstance().setAngle(0.0);
		}));

		secondary.a().whileTrue(new SetIntakeState(IntakeState.IntakingDown, intake)).whileFalse(new SetIntakeState(IntakeState.IdlingUp, intake));
		secondary.b().whileTrue(new SetIntakeState(IntakeState.OuttakingUp, intake)).whileFalse(new SetIntakeState(IntakeState.IdlingUp, intake));

		secondary.y().toggleOnTrue(new RunShooter(5500.0, 0.0, secondary, shooter));
		secondary.x().whileTrue(new RunShooter(5500.0, 5500, secondary, shooter));
		secondary.pov(90).whileTrue(new RunShooter(-5500, -5500, secondary, shooter));
		secondary.pov(180).whileTrue(new SetIntakeState(IntakeState.IntakingUp, intake));
		secondary.pov(270).whileTrue(new RunShooter(50, 500, primary, shooter));

		secondary.rightTrigger(0.1).whileTrue(new SetClimbPos(-1, 0, climb));
		secondary.leftTrigger(0.1).whileTrue(new SetClimbPos(0, -1, climb));
		secondary.rightBumper().whileTrue(new SetClimbPos(-1, 1, climb));
		secondary.leftBumper().whileTrue(new SetClimbPos(1, -1, climb));

	}

	public void checkClimb() {
		// if (DriverStation.isFMSAttached()) {
		// 	climb.initialize();
		// 	return;
		// }
		if ((Boolean) climbDown.getValue()) {
			climb.initialize();
		} else {
			climb.setEncToTop();
		}
	}

	public void configureTeleop() {
		drivetrain.setDriveFF(SwerveConstants.kFFDriveVeloTeleop);
		intake.setState(IntakeState.IdlingUp);
	}

	public void setGyroAngle() {
		// Gyro.getInstance().setAngle(drivetrain.getPose().getRotation().getDegrees());
	}

	public void setLEDMode(LEDMode mode) {
		leds.setMode(mode);
	}

	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.getSelected().andThen(new DefaultDrive(() -> 0.0, () -> 0.0, () -> 0.0, () -> false, drivetrain));
		// return new PrintCommand("Hello WOrld");
	}

}