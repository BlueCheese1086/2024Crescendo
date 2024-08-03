package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    // Motors
    private CANSparkMax launchMotor;
    private CANSparkMax feedMotor;

    // Encoders
    private RelativeEncoder launchEncoder;
    private RelativeEncoder feedEncoder;

    // PID Controllers
    private SparkPIDController launchPID;
    private SparkPIDController feedPID;

    // Logged Vars
    private double expectedFeedSpeed;
    private double expectedLaunchSpeed;

    // A common instance of the shooter class so that I don't have to outright initialize it anywhere.
    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();

        return instance;
    }

    public Shooter() {
        // Getting motors
        launchMotor = new CANSparkMax(ShooterConstants.launchID, MotorType.kBrushless);
        feedMotor = new CANSparkMax(ShooterConstants.feedID, MotorType.kBrushless);

        // Resetting motor configs
        launchMotor.restoreFactoryDefaults();
        feedMotor.restoreFactoryDefaults();

        // Setting the idle mode of motors
        launchMotor.setIdleMode(IdleMode.kCoast);
        feedMotor.setIdleMode(IdleMode.kCoast);

        // Inverting the motors
        launchMotor.setInverted(true);
        feedMotor.setInverted(true);

        // Getting the encoders
        launchEncoder = launchMotor.getEncoder();
        feedEncoder = feedMotor.getEncoder();

        // Getting the PID controllers
        launchPID = launchMotor.getPIDController();
        feedPID = feedMotor.getPIDController();

        // Setting PIDFF values
        launchPID.setP(ShooterConstants.launchP);
        launchPID.setI(ShooterConstants.launchI);
        launchPID.setD(ShooterConstants.launchD);
        launchPID.setFF(ShooterConstants.launchFF);

        launchPID.setP(ShooterConstants.feedP);
        launchPID.setI(ShooterConstants.feedI);
        launchPID.setD(ShooterConstants.feedD);
        launchPID.setFF(ShooterConstants.feedFF);

        // Saving motor configs
        launchMotor.burnFlash();
        feedMotor.burnFlash();
    }

    /** This function runs every tick that the class has been initialized. */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("/Shooter/Real_Feed_RPM", getFeedSpeed());
        SmartDashboard.putNumber("/Shooter/Expected_Feed_RPM", expectedFeedSpeed);
        SmartDashboard.putNumber("/Shooter/Real_Launch_RPM", getLaunchSpeed());
        SmartDashboard.putNumber("/Shooter/Expected_Launch_RPM", expectedLaunchSpeed);
    }

    /** Gets the speed of the feed wheel. */
    public double getFeedSpeed() {
        return feedEncoder.getVelocity();
    }

    /** Gets the speed of the launch wheel. */
    public double getLaunchSpeed() {
        return launchEncoder.getVelocity();
    }

    /** Sets the speed of the feed wheel. */
    public void setFeedSpeed(double speed) {
        expectedFeedSpeed = speed * ShooterConstants.maxFeedSpeed;
        
        feedPID.setReference(expectedFeedSpeed, ControlType.kVelocity);
    }

    /** Sets the speed of the launch wheel. */
    public void setLaunchSpeed(double speed) {
        expectedLaunchSpeed = speed * ShooterConstants.maxLaunchSpeed;
        
        launchPID.setReference(expectedLaunchSpeed, ControlType.kVelocity);
    }
}