package frc.robot.Climb;

import java.util.Objects;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimbConstants;

public class Climb {
    // A common instance of the climb subsystem.
    private static Climb instance;

    // Motors
    private CANSparkMax lTower = new CANSparkMax(ClimbConstants.lTowerID, MotorType.kBrushless);
    private CANSparkMax rTower = new CANSparkMax(ClimbConstants.rTowerID, MotorType.kBrushless);

    /**
     * Gets an instance of the Climb subsystem.
     * 
     * @return The instance of the Climb subsystem.
     */
    public static Climb getInstance() {
        // Checks if the climb system has been initialized yet.
        if (Objects.isNull(instance)) instance = new Climb();

        // Returns an instance of the Climb subsystem.
        return instance;
    }

    public Climb() {
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