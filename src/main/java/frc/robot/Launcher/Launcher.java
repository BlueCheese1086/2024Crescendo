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
    // The motors of the launch subsystem.
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

        feedMotor.setInverted(false);
        launchMotor.setInverted(false);
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
            launchPID.setReference(speed * LauncherConstants.maxLaunchSpeed, ControlType.kVelocity);
        }
    }

    /** 
     * Sets the speed of the feed motor.
     * 
     * @param speed The desired speed of the motor in %.
     */
    public void setFeedWheel(double speed) {
        // Has two different max speeds in case we use the launcher as an intake.
        if (speed > 0) {
            // Maybe try with kDutyCycle?  Would set const to 3
            // Or try setting voltage to 1.  It may have worked for Mechanical Advantage.
            feedPID.setReference(speed * LauncherConstants.maxFeedOutSpeed, ControlType.kVelocity);
        } else if (speed < 0) {
            feedPID.setReference(speed * LauncherConstants.maxFeedInSpeed, ControlType.kVelocity);
        } else {
            feedMotor.set(0);
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
        setLaunchWheel(launchSpeed);
        setFeedWheel(feedSpeed);
    }
}