// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
    // The motors of the intake subsystem.
    private CANSparkMax launchMotor = new CANSparkMax(LauncherConstants.LaunchID, MotorType.kBrushless);
    private CANSparkMax feedMotor = new CANSparkMax(LauncherConstants.FeedID, MotorType.kBrushless);

    // The PID controllers for each motor.
    private SparkPIDController launchPID = launchMotor.getPIDController();
    private SparkPIDController feedPID = feedMotor.getPIDController();

    /**
     * Creates a new Launcher subsystem.
     * 
     * This subsystem is in charge of collecting and scoring notes.
     */
    public Launcher() {
        // // Setting (and saving) defaults for each motor
        // feedMotor.restoreFactoryDefaults();
        // feedMotor.setIdleMode(IdleMode.kCoast);
        // feedMotor.setInverted(true);
        // // feedMotor.burnFlash();

        // launchMotor.restoreFactoryDefaults();
        // launchMotor.setIdleMode(IdleMode.kCoast);
        // launchMotor.setInverted(true);
        // // launchMotor.burnFlash();

        // // Setting PID Constants
        // feedPID.setP(LauncherConstants.feedP);
        // feedPID.setI(LauncherConstants.feedI);
        // feedPID.setD(LauncherConstants.feedD);
        // feedPID.setFF(LauncherConstants.feedFF);

        // launchPID.setP(LauncherConstants.launchP);
        // launchPID.setI(LauncherConstants.launchI);
        // launchPID.setD(LauncherConstants.launchD);
        // launchPID.setFF(LauncherConstants.launchFF);
        // The CANSparkMax motors are being initalized.
        feedMotor.restoreFactoryDefaults();
        launchMotor.restoreFactoryDefaults();

        feedPID.setP(0.0001);
        feedPID.setI(0);
        feedPID.setD(0);
        feedPID.setFF(0.01);

        launchPID.setP(0.0001);
        launchPID.setI(0);
        launchPID.setD(0);
        launchPID.setFF(0.01);

        feedMotor.setIdleMode(IdleMode.kCoast);
        launchMotor.setIdleMode(IdleMode.kCoast);

        feedMotor.setInverted(true);
        launchMotor.setInverted(true);
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
        if (speed == 0) {
            feedMotor.set(0);
        } else {
            feedPID.setReference(speed * LauncherConstants.feedSpeed, ControlType.kVelocity);
        }
    }

    /**
     * Sets the speed of the launch and feed motors.
     * 
     * @param launchSpeed The desired speed of the launch motor in RPM
     * @param feedSpeed The desired speed of the feed motor in RPM
     */
    public void setSpeeds(double launchSpeed, double feedSpeed) {
        // Setting speeds for each motor
        // launchPID.setReference(launchSpeed, ControlType.kVelocity);
        // feedPID.setReference(feedSpeed, ControlType.kVelocity);

        // This code worked, be careful using the above
        // Setting speeds for each motor
        setLaunchWheel(launchSpeed);
        setFeedWheel(feedSpeed);
    }
}