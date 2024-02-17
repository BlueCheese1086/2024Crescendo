// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Launcher;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
    // The motors of the intake subsystem.
    private CANSparkMax launchMotor = new CANSparkMax(LauncherConstants.LauncherID, MotorType.kBrushless);
    private CANSparkMax feedMotor = new CANSparkMax(LauncherConstants.FeederID, MotorType.kBrushless);

    // The encoders for each motor.
    private RelativeEncoder launchEncoder = launchMotor.getEncoder();
    private RelativeEncoder feedEncoder = feedMotor.getEncoder();

    // The PID controllers for each motor.
    private SparkPIDController launchPID = launchMotor.getPIDController();
    private SparkPIDController feedPID = feedMotor.getPIDController();

    /**
     * Creates a new Launcher subsystem.
     * 
     * This subsystem is in charge of collecting and scoring notes.
     */
    public Launcher() {
        // Restoring default settings to the motors.
        feedMotor.restoreFactoryDefaults();
        launchMotor.restoreFactoryDefaults();

        // Setting the idle mode of the motors.
        feedMotor.setIdleMode(IdleMode.kCoast);
        launchMotor.setIdleMode(IdleMode.kCoast);

        // Inverting the motors.
        feedMotor.setInverted(true);
        launchMotor.setInverted(true);

        // Setting velocity conversion factors
        feedEncoder.setVelocityConversionFactor(LauncherConstants.feedConversionFactor / 60);
        launchEncoder.setVelocityConversionFactor(LauncherConstants.launchConversionFactor / 60);

        // Setting feed PID values
        feedPID.setP(LauncherConstants.feedP);
        feedPID.setI(LauncherConstants.feedI);
        feedPID.setD(LauncherConstants.feedD);
        feedPID.setFF(LauncherConstants.feedFF);

        // Setting launch PID values
        launchPID.setP(LauncherConstants.launchP);
        launchPID.setI(LauncherConstants.launchI);
        launchPID.setD(LauncherConstants.launchD);
        launchPID.setFF(LauncherConstants.launchFF);

        // Saving settings for the motors.
        feedMotor.burnFlash();
        launchMotor.burnFlash();
    }

    /**
     * Sets the speed of the launch motor to a value in RPM.
     * 
     * @param speed The desired speed of the motor in RPM.
     */
    public void setLaunchWheel(double speed) {
        if (speed == 0) {
            launchMotor.set(0);
        } else {
            launchPID.setReference(speed * LauncherConstants.launchSpeed, ControlType.kVelocity);
        }
    }

    /**
     * Sets the speed of the feed motor to a value in RPM.
     * 
     * @param speed The desired speed of the motor in RPM.
     */
    public void setFeedWheel(double speed) {
        if (speed == 0 || launchEncoder.getVelocity() < 0) {
            feedMotor.set(0);
        } else {
            feedPID.setReference(speed * LauncherConstants.feedSpeed, ControlType.kVelocity);
        }
    }

    /**
     * Sets the speed of the launch and feed motors.
     * 
     * @param launchSpeed The desired speed of the launch motor in RPM
     * @param feedSpeed   The desired speed of the feed motor in RPM
     */
    public void setSpeeds(double launchSpeed, double feedSpeed) {
        // Setting speeds for each motor
        setLaunchWheel(launchSpeed);
        setFeedWheel(feedSpeed);
    }
}