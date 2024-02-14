package frc.robot.Climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
    // Motors
    private CANSparkMax left = new CANSparkMax(ClimbConstants.leftID, MotorType.kBrushless);
    private CANSparkMax right = new CANSparkMax(ClimbConstants.rightID, MotorType.kBrushless);

    // Encoders
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    public Climb() {
        // Resetting the settings of the motors.
        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        // Setting an amp limit.
        left.setSmartCurrentLimit(35);
        right.setSmartCurrentLimit(35);

        // Enabling voltage compansation.
        left.enableVoltageCompensation(12);
        right.enableVoltageCompensation(12);

        // Inverting the motors as necessary.
        left.setInverted(false);
        right.setInverted(true);

        // Setting idle modes for each motor.
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kCoast);

        // Getting the encoders of each motor.
        leftEncoder = left.getEncoder();
        rightEncoder = right.getEncoder();

        // Resetting encoder positions.
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        // Setting RPM conversions for each encoder.
        leftEncoder.setPositionConversionFactor(360.0 / ClimbConstants.leftRatio); // To degrees
        leftEncoder.setVelocityConversionFactor(1 / ClimbConstants.leftRatio / 60.0); // To M/S
        rightEncoder.setPositionConversionFactor(1 / ClimbConstants.rightRatio); // To M
        rightEncoder.setVelocityConversionFactor(1 / ClimbConstants.rightRatio / 60.0); // To M/S

        // Saving the settings.
        left.burnFlash();
        right.burnFlash();
    }

    /**
     * Sets the speed of the right tower in % speed
     * 
     * @param speed The speed to set the right to in % speed
     */
    public void setRightSpeed(double speed) {
        right.set(speed * ClimbConstants.maxSpeed);
    }

    /**
     * Sets the speed of the left tower in % speed
     * 
     * @param speed The speed to set the right to in % speed
     */
    public void setLeftSpeed(double speed) {
        left.set(speed * ClimbConstants.maxSpeed);
    }
}