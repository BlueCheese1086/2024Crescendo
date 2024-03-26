package frc.robot.Shooter;

import java.util.Objects;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ShooterConstants;

public class Shooter {
    // motors
    // this code was brought to you by lil' python
    private TalonFX lShooter = new TalonFX(ShooterConstants.lShooterID);
    private TalonFX rShooter = new TalonFX(ShooterConstants.rShooterID);

    private CANSparkMax feedRoller = new CANSparkMax(ShooterConstants.feedRollerID, MotorType.kBrushless);

    // A common instance of the shooter subsystem.
    private static Shooter instance;

    //🫵🤫
    public enum State {
        IDLE,
        SPINNINGUPANDFEEDING,
        SHOOTING,
        SPINNINGUPNOFEEDING,
        WEAKSHOT
    }

    public Shooter() {
        // Creating the configuration objects for the talons
        TalonFXConfiguration lShooterConfig = new TalonFXConfiguration();
        TalonFXConfiguration rShooterConfig = new TalonFXConfiguration();

        // Setting the idle mode of the talons
        lShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Updating the settings of the talons
        lShooter.getConfigurator().apply(lShooterConfig);
        rShooter.getConfigurator().apply(rShooterConfig);

        // Making the rShooter motor follow the lShooter motor
        rShooter.setControl(new Follower(lShooter.getDeviceID(), true));

        // Resetting the settings of the sparkmaxes
        feedRoller.restoreFactoryDefaults();

        // Setting the idle mode of the sparkmaxes
        feedRoller.setIdleMode(IdleMode.kBrake);

        // Saving the settings of the sparkmaxes
        feedRoller.burnFlash();
    }

    /**
     * This function gets a common instance of the shooter subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Shooter subsystem.
     */
    public static Shooter getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (Objects.isNull(instance)) instance = new Shooter();

        return instance;
    }

    /**
     * Sets the speeds of the left and right shooter motors.
     * 
     * @param speed The speed of the motors in duty cycle output.
     */
    public void runLauncher(double speed) {
        lShooter.set(speed * ShooterConstants.maxShootSpeed);
    }

    /**
     * Sets the speed of the feed rollers.
     * 
     * @param speed The speed of the motor in duty cycle output.
     */
    public void runFeed(double speed) {
        feedRoller.set(speed * ShooterConstants.maxFeedSpeed);
    }
}