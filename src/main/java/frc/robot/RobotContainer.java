// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.RobotMap.Climber;
import frc.robot.Climb.Climb;
import frc.robot.Climb.ClimbIOReplay;
import frc.robot.Climb.ClimbIOSparkMax;
import frc.robot.Climb.ClimbIOSim;
import frc.robot.Drive.Drive;
import frc.robot.Drive.GyroIOPigeon2Phoenix6;
import frc.robot.Drive.GyroIOReplay;
import frc.robot.Drive.ModuleIOReal;
import frc.robot.Drive.ModuleIOReplay;
import frc.robot.Drive.ModuleIOSim;
import frc.robot.Feeder.BeambreakIOReal;
import frc.robot.Feeder.BeambreakIOReplay;
import frc.robot.Feeder.BeambreakIOSim;
import frc.robot.Feeder.Feeder;
import frc.robot.Feeder.FeederIOReplay;
import frc.robot.Feeder.FeederIOSim;
import frc.robot.Feeder.FeederIOSparkMax;
import frc.robot.Intake.Intake;
import frc.robot.Intake.IntakeIOReplay;
import frc.robot.Intake.IntakeIOSim;
import frc.robot.Intake.IntakeIOSparkMax;
import frc.robot.Pivot.Pivot;
import frc.robot.Pivot.PivotIOReplay;
import frc.robot.Pivot.PivotIOSim;
import frc.robot.Pivot.PivotIOSparkMax;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.ShooterIOReplay;
import frc.robot.Shooter.ShooterIOSim;
import frc.robot.Shooter.ShooterIOSparkMax;
import swervelib.SwerveDrive;

public class RobotContainer {
    // Initializing the subsystems
    private final Drive drive;
    private final Climb climber;
    private final Intake intake;
    private final Feeder feeder;
    private final Pivot pivot;
    private final Shooter shooter;
    private final Visualizer visualizer;

    // Creating the controllers
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController operator = new CommandXboxController(1);

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                    new GyroIOPigeon2Phoenix6(),
                    new ModuleIOReal(0),
                    new ModuleIOReal(1),
                    new ModuleIOReal(2),
                    new ModuleIOReal(3)
                );
                
                climber = new Climb(new ClimbIOSparkMax());
                intake = new Intake(new IntakeIOSparkMax());
                feeder = new Feeder(new FeederIOSparkMax(), new BeambreakIOReal(RobotMap.Shooter.feederBeambreak), new BeambreakIOReal(RobotMap.Shooter.shooterBeambreak));
                pivot = new Pivot(new PivotIOSparkMax());
                shooter = new Shooter(new ShooterIOSparkMax());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                    new GyroIOReplay() {},
                    new ModuleIOSim("FrontLeft"),
                    new ModuleIOSim("FrontRight"),
                    new ModuleIOSim("BackLeft"),
                    new ModuleIOSim("BackRight")
                );

                climber = new Climb(new ClimbIOSim());
                intake = new Intake(new IntakeIOSim());
                feeder = new Feeder(new FeederIOSim(), new BeambreakIOSim(RobotMap.Shooter.feederBeambreak), new BeambreakIOSim(RobotMap.Shooter.shooterBeambreak));
                pivot = new Pivot(new PivotIOSim());
                shooter = new Shooter(new ShooterIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                    new GyroIOReplay() {},
                    new ModuleIOReplay() {},
                    new ModuleIOReplay() {},
                    new ModuleIOReplay() {},
                    new ModuleIOReplay() {}
                );

                climber = new Climb(new ClimbIOReplay());
                intake = new Intake(new IntakeIOReplay());
                feeder = new Feeder(new FeederIOReplay(), new BeambreakIOReplay(), new BeambreakIOReplay());
                pivot = new Pivot(new PivotIOReplay());
                shooter = new Shooter(new ShooterIOReplay());
                break;

        }

        visualizer = new Visualizer(climber, intake, pivot);

        configureBindings();
    }

    private void configureBindings() {
        /*  All operator controls. -- See Controls.png for visual reference.
            A: Runs the feed
            B: Runs the intake
            X: Runs the launcher in reverse
            Y: Toggles the launcher
            Left Bumper: Moves the shooter up.
            Left Trigger: Moves the shooter down.
            Right Bumper: Moves the tower up.
            Right Trigger: Moves the tower down.
            Back: Running the intake in reverse.
            D-Pad Up: Stowing the intake.
            D-Pad Down: Extending the intake.
            Left Joystick: Move up/down/left/right respectively
            Right Joystick: Turn left/right respectively
        */
        // Driver Controller

        // left trigger -> climb up
        driver.leftTrigger(0.1).onTrue(
            climber.setDutyCycle(1)
        ).onFalse(
            climber.setDutyCycle(0)
        );

        // right trigger -> clinb up
        driver.rightTrigger(0.1).onTrue(
            climber.setDutyCycle(-1)
        ).onFalse(
            climber.setDutyCycle(0)
        );

        // Operator Controller

        // D-Pad Up for intake down, rollers forward, until note in feeder beambreak
        operator.povUp().whileTrue(
            Commands.parallel(
                intake.setIntakeDown(false),
                feeder.setRPM(() -> 2000)
            ).until(
                () -> feeder.feederBeambreakObstructed()
            )
        ).onFalse(
            intake.setIntakeUp()
        );

        // D-Pad Down for intake down, rollers backward
        operator.povDown().whileTrue(
            Commands.parallel(
                intake.setIntakeDown(true),
                feeder.setRPM(() -> -2000)
            )
        ).onFalse(
            intake.setIntakeUp()
        );

        // Right trigger for run intake forward
        operator.rightTrigger(0.1).whileTrue(
            Commands.parallel(
                intake.setRollerRPM(() -> 2000),
                feeder.setRPM(() -> 0)).until(() -> feeder.feederBeambreakObstructed()
            )
        ).onFalse(
            Commands.parallel(
                intake.setRollerRPM(() -> 0),
                feeder.setRPM(() -> 0)
            )
        );

        // Right bumper for run intake backward
        operator.rightBumper().whileTrue(
            Commands.parallel(
                intake.setRollerRPM(() -> -2000),
                feeder.setRPM(() -> -2000)
            )
        ).onFalse(
            Commands.parallel(
                intake.setRollerRPM(() -> 0),
                feeder.setRPM(() -> 0)
            )
        );

        // Y for shooter at subwoofer
        operator.y().onTrue(
            Commands.parallel(
                pivot.setPivotTarget(() -> Units.radiansToDegrees(56)),
                shooter.setRPM(() -> 5800, 0.3)
            ).andThen(
                feeder.setRPM(() -> 2000)
                .until(
                    () -> (!feeder.feederBeambreakObstructed() && !feeder.shooterBeambreakObstructed())
                )
            )
        );

        // X for shooter at amp

        // B for shooter at podium or feeding

        // A for shooter at source
        operator.a().onTrue(
            Commands.parallel(
                pivot.setPivotTarget(() -> Units.radiansToDegrees(56)),
                shooter.setRPM(() -> -2000, 1.0),
                feeder.setRPM(() -> -2000)
            ).andThen(
                feeder.setRPM(() -> 2000)
                .until(
                    () -> (feeder.feederBeambreakObstructed() && !feeder.shooterBeambreakObstructed())
                )
            )
        );

        // driver.povLeft().onTrue(new PlayMusic("jingle_bells.chrp"));
    }

    public Command getAutonomousCommand() {
        return new PrintCommand("No auto lol");// PathPlannerAuto("V3-Auto");
    }

    private static double teleopAxisAdjustment(double x) {
        return MathUtil.applyDeadband(Math.abs(Math.pow(x, 2)) * Math.signum(x), 0.02);
    }

    public Command getTeleopCommand() {
        return drive.runVoltageTeleopFieldRelative(
            () -> new ChassisSpeeds(
                -teleopAxisAdjustment( driver.getLeftY()) * DriveConstants.maxLinearVelocity,
                -teleopAxisAdjustment( driver.getLeftX()) * DriveConstants.maxLinearVelocity,
                -teleopAxisAdjustment(driver.getRightX()) * DriveConstants.maxLinearVelocity
            )
        );
    }
}
