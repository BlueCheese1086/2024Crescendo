// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.YAGSLConverter.Configs;

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

        Configs.SwerveDriveJson.IMU.type = "pigeon2";
        Configs.SwerveDriveJson.IMU.id = DriveConstants.gyroID;
        Configs.SwerveDriveJson.invertedIMU = false;

        Configs.FrontLeftModuleJson.absoluteEncoderInverted = false;
        Configs.FrontLeftModuleJson.absoluteEncoderOffset = DriveConstants.flOffset;
        Configs.FrontLeftModuleJson.Angle.id = DriveConstants.flTurnID;
        Configs.FrontLeftModuleJson.Angle.type = "sparkmax";
        Configs.FrontLeftModuleJson.Drive.id = DriveConstants.flDriveID;
        Configs.FrontLeftModuleJson.Drive.type = "sparkmax";
        Configs.FrontLeftModuleJson.Encoder.id = DriveConstants.flCancoderID;
        Configs.FrontLeftModuleJson.Encoder.type = "ctre_mag";
        Configs.FrontLeftModuleJson.Inverted.angle = false;
        Configs.FrontLeftModuleJson.Inverted.drive = true;
        Configs.FrontLeftModuleJson.Location.front = DriveConstants.length / 2;
        Configs.FrontLeftModuleJson.Location.left = DriveConstants.width / 2;

        Configs.FrontRightModuleJson.absoluteEncoderInverted = false;
        Configs.FrontRightModuleJson.absoluteEncoderOffset = DriveConstants.frOffset;
        Configs.FrontRightModuleJson.Angle.id = DriveConstants.frTurnID;
        Configs.FrontRightModuleJson.Angle.type = "sparkmax";
        Configs.FrontRightModuleJson.Drive.id = DriveConstants.frDriveID;
        Configs.FrontRightModuleJson.Drive.type = "sparkmax";
        Configs.FrontRightModuleJson.Encoder.id = DriveConstants.frCancoderID;
        Configs.FrontRightModuleJson.Encoder.type = "ctre_mag";
        Configs.FrontRightModuleJson.Inverted.angle = false;
        Configs.FrontRightModuleJson.Inverted.drive = true;
        Configs.FrontRightModuleJson.Location.front = -DriveConstants.length / 2;
        Configs.FrontRightModuleJson.Location.left = DriveConstants.width / 2;

        Configs.BackLeftModuleJson.absoluteEncoderInverted = false;
        Configs.BackLeftModuleJson.absoluteEncoderOffset = DriveConstants.blOffset;
        Configs.BackLeftModuleJson.Angle.id = DriveConstants.blTurnID;
        Configs.BackLeftModuleJson.Angle.type = "sparkmax";
        Configs.BackLeftModuleJson.Drive.id = DriveConstants.blDriveID;
        Configs.BackLeftModuleJson.Drive.type = "sparkmax";
        Configs.BackLeftModuleJson.Encoder.id = DriveConstants.blCancoderID;
        Configs.BackLeftModuleJson.Encoder.type = "ctre_mag";
        Configs.BackLeftModuleJson.Inverted.angle = false;
        Configs.BackLeftModuleJson.Inverted.drive = true;
        Configs.BackLeftModuleJson.Location.front = DriveConstants.length / 2;
        Configs.BackLeftModuleJson.Location.left = -DriveConstants.width / 2;

        Configs.BackRightModuleJson.absoluteEncoderInverted = false;
        Configs.BackRightModuleJson.absoluteEncoderOffset = DriveConstants.brOffset;
        Configs.BackRightModuleJson.Angle.id = DriveConstants.brTurnID;
        Configs.BackRightModuleJson.Angle.type = "sparkmax";
        Configs.BackRightModuleJson.Drive.id = DriveConstants.brDriveID;
        Configs.BackRightModuleJson.Drive.type = "sparkmax";
        Configs.BackRightModuleJson.Encoder.id = DriveConstants.brCancoderID;
        Configs.BackRightModuleJson.Encoder.type = "ctre_mag";
        Configs.BackRightModuleJson.Inverted.angle = false;
        Configs.BackRightModuleJson.Inverted.drive = true;
        Configs.BackRightModuleJson.Location.front = -DriveConstants.length / 2;
        Configs.BackRightModuleJson.Location.left = -DriveConstants.width / 2;

        Configs.PhysicalPropertiesJson.optimalVoltage = 12;
        Configs.PhysicalPropertiesJson.wheelGripCoefficientOfFriction = 1;
        Configs.PhysicalPropertiesJson.ConversionFactors.Angle.factor = DriveConstants.turnPosConversionFactor;
        Configs.PhysicalPropertiesJson.ConversionFactors.Angle.gearRatio = DriveConstants.turnRatio;
        Configs.PhysicalPropertiesJson.ConversionFactors.Drive.diameter = DriveConstants.wheelRadius * 2;
        Configs.PhysicalPropertiesJson.ConversionFactors.Drive.factor = DriveConstants.drivePosConversionFactor;
        Configs.PhysicalPropertiesJson.ConversionFactors.Drive.gearRatio = DriveConstants.driveRatio;
        Configs.PhysicalPropertiesJson.CurrentLimit.angle = 40;
        Configs.PhysicalPropertiesJson.CurrentLimit.drive = 40;
        Configs.PhysicalPropertiesJson.RampRate.angle = 12;
        Configs.PhysicalPropertiesJson.RampRate.drive = 12;

        Configs.PIDFPropertiesJson.Angle.integralZone = 0;
        Configs.PIDFPropertiesJson.Angle.kD = DriveConstants.turnD;
        Configs.PIDFPropertiesJson.Angle.kI = DriveConstants.turnI;
        Configs.PIDFPropertiesJson.Angle.kP = DriveConstants.turnP;
        Configs.PIDFPropertiesJson.Angle.staticFeedForward = DriveConstants.turnFF;
        Configs.PIDFPropertiesJson.Drive.integralZone = 0;
        Configs.PIDFPropertiesJson.Drive.kD = DriveConstants.driveD;
        Configs.PIDFPropertiesJson.Drive.kI = DriveConstants.driveI;
        Configs.PIDFPropertiesJson.Drive.kP = DriveConstants.driveP;
        Configs.PIDFPropertiesJson.Drive.staticFeedForward = DriveConstants.driveFF;
        Configs.PIDFPropertiesJson.Drive.max = DriveConstants.maxDriveSpeed;
        Configs.PIDFPropertiesJson.Drive.min = -DriveConstants.maxDriveSpeed;
        Configs.PIDFPropertiesJson.Angle.max = DriveConstants.maxTurnSpeed;
        Configs.PIDFPropertiesJson.Angle.min = -DriveConstants.maxTurnSpeed;

        new YAGSLConverter().writeConfigs();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}