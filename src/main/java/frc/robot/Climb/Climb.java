package frc.robot.Climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;

public class Climb extends SubsystemBase {
    // Motor
    private CANSparkMax climbMotor;

    // Encoder
    private RelativeEncoder climbEncoder;

    // A common instance of the tower subsystem.
    private static Climb instance;

    /**
     * This function gets a common instance of the tower subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Tower subsystem.
     */
    public static Climb getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (instance == null) instance = new Climb();

        return instance;
    }

    public Climb() {
        // Initializing the motor
        climbMotor = new CANSparkMax(TowerConstants.lTowerID, MotorType.kBrushless);

        // Resetting the settings on the sparkmax
        climbMotor.restoreFactoryDefaults();

        // Setting the idle modes of the sparkmax
        climbMotor.setIdleMode(IdleMode.kBrake);

        // Getting the climb encoder
        climbEncoder = climbMotor.getEncoder();

        // Saving the settings to the sparkmax
        climbMotor.burnFlash();
    }

    /**
     * Sets the speed of the tower.
     * 
     * @param speed The duty cycle speed of the tower.
     */
    public void setSpeed(double speed) {
        climbMotor.set(speed);
    }

    /**
     * Gets the speed of the tower.
     * 
     * @return The duty cycle speed of the tower.
     */
    public double getSpeed() {
        return climbMotor.get();
    }

    /**
     * Gets whether or not the tower has reached its max or min height.
     * 
     * @return Whether or not the tower has reached its max or min height.
     */
    public boolean atLimits() {
        return climbMotor.get() != 0 && climbEncoder.getVelocity() < 0.1;
    }
}