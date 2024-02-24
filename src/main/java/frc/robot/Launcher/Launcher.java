// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Launcher;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
    // The motors of the intake subsystem.
    private CANSparkMax launchMotor = new CANSparkMax(LauncherConstants.LauncherID, MotorType.kBrushless);
    private CANSparkMax feedMotor = new CANSparkMax(LauncherConstants.FeederID, MotorType.kBrushless);

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
        launchMotor.set(speed);
    }

    /**
     * Sets the speed of the feed motor to a value in RPM.
     * 
     * @param speed The desired speed of the motor in RPM.
     */
    public void setFeedWheel(double speed) {
        if (launchMotor.get() >= 0) {
            feedMotor.set(speed);
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