package frc.robot.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    // Motors
    private CANSparkMax lShooter = new CANSparkMax(ShooterConstants.lShooterID, MotorType.kBrushless);
    private CANSparkMax rShooter = new CANSparkMax(ShooterConstants.rShooterID, MotorType.kBrushless);
    private CANSparkMax feedRoller = new CANSparkMax(ShooterConstants.feedRollerID, MotorType.kBrushless);

    // A common instance of the shooter subsystem.
    private static Shooter instance;

    /**
     * This function gets a common instance of the shooter subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Shooter subsystem.
     */
    public static Shooter getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (instance == null) instance = new Shooter();

        return instance;
    }

    public Shooter() {
        // Resetting the settings of the sparkmaxes
        lShooter.restoreFactoryDefaults();
        rShooter.restoreFactoryDefaults();
        feedRoller.restoreFactoryDefaults();

        // Setting the idle mode of the sparkmaxes
        lShooter.setIdleMode(IdleMode.kBrake);
        rShooter.setIdleMode(IdleMode.kBrake);
        feedRoller.setIdleMode(IdleMode.kBrake);

        // Making the rShooter motor follow the lShooter motor
        rShooter.follow(lShooter, true);
        feedRoller.setInverted(true);
        lShooter.setInverted(true);

        // Saving the settings of the sparkmaxes
        lShooter.burnFlash();
        rShooter.burnFlash();
        feedRoller.burnFlash();
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