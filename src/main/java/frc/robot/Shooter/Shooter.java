package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private CANSparkMax launcher;
    private CANSparkMax feeder;

    private RelativeEncoder launcherEncoder;
    private RelativeEncoder feederEncoder;

    private double expectedFeedSpeed;
    private double expectedLaunchSpeed;

    public Shooter() {
        launcher = new CANSparkMax(ShooterConstants.launchID, MotorType.kBrushless);
        feeder = new CANSparkMax(ShooterConstants.feedID, MotorType.kBrushless);

        launcher.restoreFactoryDefaults();
        feeder.restoreFactoryDefaults();

        launcher.setIdleMode(IdleMode.kCoast);
        feeder.setIdleMode(IdleMode.kCoast);

        launcher.setInverted(true);
        feeder.setInverted(true);

        launcherEncoder = launcher.getEncoder();
        feederEncoder = feeder.getEncoder();

        launcher.burnFlash();
        feeder.burnFlash();
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
        feeder.set(speed * ShooterConstants.maxFeedSpeed);
    }

    public void setLaunchSpeed(double speed) {
        expectedLaunchSpeed = speed;
        launcher.set(speed * ShooterConstants.maxLaunchSpeed);
    }
}