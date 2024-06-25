package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private CANSparkMax launcher;
    private CANSparkMax feeder;

    public Shooter() {
        launcher = new CANSparkMax(ShooterConstants.launchID, MotorType.kBrushless);
        feeder = new CANSparkMax(ShooterConstants.feedID, MotorType.kBrushless);

        launcher.restoreFactoryDefaults();
        feeder.restoreFactoryDefaults();

        launcher.setIdleMode(IdleMode.kCoast);
        feeder.setIdleMode(IdleMode.kCoast);

        launcher.setInverted(true);
        feeder.setInverted(true);

        launcher.burnFlash();
        feeder.burnFlash();
    }

    public void setFeedSpeed(double speed) {
        feeder.set(speed * ShooterConstants.maxFeedSpeed);
    }

    public void setLaunchSpeed(double speed) {
        launcher.set(speed * ShooterConstants.maxLaunchSpeed);
    }
}
