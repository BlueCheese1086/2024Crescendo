package frc.robot.Tower;

import java.util.Objects;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.TowerConstants;

public class Tower {
    // Motors
    private CANSparkMax tower;

    // A common instance of the tower subsystem.
    private static Tower instance;

    public Tower() {
        // Initializing the motor
        tower = new CANSparkMax(TowerConstants.lTowerID, MotorType.kBrushless);

        // Resetting the settings on the sparkmaxes
        tower.restoreFactoryDefaults();

        // Setting the idle modes of the sparkmaxs
        tower.setIdleMode(IdleMode.kBrake);

        // Inverting the motors
        tower.setInverted(false);

        // Saving the settings to the sparkmax
        tower.burnFlash();
    }

    /**
     * This function gets a common instance of the tower subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Tower subsystem.
     */
    public static Tower getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (Objects.isNull(instance)) instance = new Tower();

        return instance;
    }

    /**
     * Sets the speed of the tower.
     * 
     * @param speed The duty cycle speed of the tower.
     */
    public void setSpeed(double speed) {
        tower.set(speed);
    }

    /**
     * Gets the speed of the tower.
     * 
     * @return The duty cycle speed of the tower.
     */
    public double getSpeed() {
        return tower.get();
    }
}