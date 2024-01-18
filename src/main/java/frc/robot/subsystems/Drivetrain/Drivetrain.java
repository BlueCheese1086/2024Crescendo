package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.FrontLeftID, MotorType.kBrushless);
    CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.BackLeftID, MotorType.kBrushless);
    CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.FrontRightID, MotorType.kBrushless);
    CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.BackRightID, MotorType.kBrushless);

    /**
     * Constructor. This method is called when an instance of the class is created. This should generally be used to set up
     * instance variables and perform any configuration or necessary set up on hardware.
     */
    public Drivetrain() {
        /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
        *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */
        frontLeftMotor.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
        backLeftMotor.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
        frontRightMotor.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
        backRightMotor.setSmartCurrentLimit(DriveConstants.kCurrentLimit);

        // Set the rear motors to follow the front motors.
        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);

        // Invert the left side so both side drive forward with positive motor outputs
        frontLeftMotor.setInverted(true);
    }

    /** Drives the robot with the y axis of one joystick and the x axis of another.  Drives robots in a way similar to how most games are played. */
    public void arcadeDrive(double xSpeed, double zRotation) {
        // Applies a deadband to the inputs.
        MathUtil.applyDeadband(xSpeed, DriveConstants.deadband);
        MathUtil.applyDeadband(zRotation, DriveConstants.deadband);

        // Square the inputs (while preserving the sign) to increase fine control while permitting full power.
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        zRotation = Math.copySign(zRotation * zRotation, zRotation);

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