package frc.robot.Tower;

import java.util.Objects;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.TowerConstants;

public class Tower {
    // Motors
    private CANSparkMax lTower = new CANSparkMax(TowerConstants.lTowerID, MotorType.kBrushless);
    private CANSparkMax rTower = new CANSparkMax(TowerConstants.rTowerID, MotorType.kBrushless);

    // A common instance of the tower subsystem.
    private static Tower instance;

    public Tower() {
        // Resetting the settings on the sparkmaxes
        lTower.restoreFactoryDefaults();
        rTower.restoreFactoryDefaults();

        // Setting the idle modes of the sparkmaxs
        lTower.setIdleMode(IdleMode.kBrake);
        rTower.setIdleMode(IdleMode.kBrake);

        // Inverting the motors
        lTower.setInverted(false);
        rTower.setInverted(true);

        // Saving the settings to the sparkmax
        lTower.burnFlash();
        rTower.burnFlash();
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
     * Sets the speed of the left tower.
     * 
     * @param speed The duty cycle speed of the left tower.
     */
    public void setLeftSpeed(double speed) {
        lTower.set(speed);
    }

    /**
     * Sets the speed of the right tower.
     * 
     * @param speed The duty cycle speed of the right tower.
     */
    public void setRightSpeed(double speed) {
        rTower.set(speed);
    }

    /**
     * Gets the speed of the left tower.
     * 
     * @return The duty cycle speed of the left tower.
     */
    public double getLeftSpeed() {
        return lTower.get();
    }
    /**
     * Gets the speed of the right tower.
     * 
     * @return The duty cycle speed of the right tower.
     */
    public double getRightSpeed() {
        return rTower.get();
    }
}