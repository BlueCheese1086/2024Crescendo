package frc.robot.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    // The motors for the drivetrain subsystem
    CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.FrontLeftID, MotorType.kBrushless);
    CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.BackLeftID, MotorType.kBrushless);
    CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.FrontRightID, MotorType.kBrushless);
    CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.BackRightID, MotorType.kBrushless);

    /**
     * Constructor. This method is called when an instance of the class is created. This should generally be used to set up
     * instance variables and perform any configuration or necessary set up on hardware.
     */
    public Drivetrain() {
        // Setting the default settings for each motor
        frontLeftMotor.restoreFactoryDefaults();
        frontLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
        frontLeftMotor.setInverted(true);
        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        frontLeftMotor.burnFlash();

        backLeftMotor.restoreFactoryDefaults();
        backLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
        backLeftMotor.setInverted(true);
        backLeftMotor.setIdleMode(IdleMode.kBrake);
        backLeftMotor.follow(frontLeftMotor);
        backLeftMotor.burnFlash();

        frontRightMotor.restoreFactoryDefaults();
        frontRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
        frontRightMotor.setInverted(false);
        frontRightMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.burnFlash();

        backRightMotor.restoreFactoryDefaults();
        backRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
        backRightMotor.setInverted(false);
        backRightMotor.setIdleMode(IdleMode.kBrake);
        backRightMotor.follow(frontRightMotor);
        backRightMotor.burnFlash();
    }

    /**
     * Drives the robot at the desired forward speed combined with the rotational speed.
     * 
     * @param xSpeed The desired forward speed of the robot.
     * @param zRotation The desired rotational speed of the robot.
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        // Applies a deadband to the inputs.
        MathUtil.applyDeadband(xSpeed, DriveConstants.deadband);
        MathUtil.applyDeadband(zRotation, DriveConstants.deadband);

        // Squares the inputs (while preserving the sign) to increase fine control while permitting full power.
        xSpeed = Math.abs(xSpeed) * xSpeed;
        zRotation = Math.abs(zRotation) * zRotation;

        // Creates the saturated speeds of the motors
        double leftSpeed = xSpeed - zRotation;
        double rightSpeed = xSpeed + zRotation;

        // Finds the maximum possible value of throttle + turn along the vector that the joystick is pointing, and then desaturates the wheel speeds.
        double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
        double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotation));
        if (greaterInput == 0.0) {
            leftSpeed = 0;
            rightSpeed = 0;
        } else {
            double saturatedInput = (greaterInput + lesserInput) / greaterInput;
            leftSpeed /= saturatedInput;
            rightSpeed /= saturatedInput;
        }

        // Sets the speed of the motors.
        frontLeftMotor.set(leftSpeed * DriveConstants.maxSpeed);
        frontRightMotor.set(rightSpeed * DriveConstants.maxSpeed);
    }
}