package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    // Motors
    private CANSparkMax launcherMotor;
    private CANSparkMax feederMotor;

    // Encoders
    private RelativeEncoder launcherEncoder;
    private RelativeEncoder feederEncoder;

    // PID Controllers
    private SparkPIDController launcherPID;
    private SparkPIDController feederPID;

    // Logged Vars
    private double expectedFeedSpeed;
    private double expectedLaunchSpeed;

    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();

        return instance;
    }

    public Shooter() {
        // Getting motors
        launcherMotor = new CANSparkMax(ShooterConstants.launchID, MotorType.kBrushless);
        feederMotor = new CANSparkMax(ShooterConstants.feedID, MotorType.kBrushless);

        // Resetting motor configs
        launcherMotor.restoreFactoryDefaults();
        feederMotor.restoreFactoryDefaults();

        // Setting the idle mode of motors
        launcherMotor.setIdleMode(IdleMode.kCoast);
        feederMotor.setIdleMode(IdleMode.kCoast);

        // Inverting the motors
        launcherMotor.setInverted(true);
        feederMotor.setInverted(true);

        // Getting the encoders
        launcherEncoder = launcherMotor.getEncoder();
        feederEncoder = feederMotor.getEncoder();

        // Getting the PID controllers
        launcherPID = launcherMotor.getPIDController();
        feederPID = feederMotor.getPIDController();

        // Saving motor configs
        launcherMotor.burnFlash();
        feederMotor.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("/Shooter/Real_Feed_RPM", getFeedSpeed());
        SmartDashboard.putNumber("/Shooter/Real_Launch_RPM", getLaunchSpeed());

        SmartDashboard.putNumber("/Shooter/Expected_Feed_RPM", expectedFeedSpeed * ShooterConstants.maxRPM);
        SmartDashboard.putNumber("/Shooter/Expected_Launch_RPM", expectedLaunchSpeed * ShooterConstants.maxRPM);
    }

    public double getFeedSpeed() {
        return feederEncoder.getVelocity();
    }

    public double getLaunchSpeed() {
        return launcherEncoder.getVelocity();
    }

    public void setFeedSpeed(double speed) {
        expectedFeedSpeed = speed;
        feederMotor.set(speed * ShooterConstants.maxFeedSpeed);
    }

    public void setLaunchSpeed(double speed) {
        expectedLaunchSpeed = speed;
        launcherMotor.set(speed * ShooterConstants.maxLaunchSpeed);
    }
}