package frc.robot.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    // Enum for setpoints
    public enum States {
        OPEN(-0.25),
        CLOSED(0);

        public final double value;

        States(double value) {
            this.value = value;
        }
    }

    // Motors
    private CANSparkMax access = new CANSparkMax(IntakeConstants.accessID, MotorType.kBrushless);
    private CANSparkMax roller = new CANSparkMax(IntakeConstants.rollerID, MotorType.kBrushless);

    // Encoders
    private RelativeEncoder accessEncoder;
    private RelativeEncoder rollerEncoder;

    // PIDs
    private SparkPIDController accessPID;

    public Intake() {
        // Resetting the settings of the motors.
        access.restoreFactoryDefaults();
        roller.restoreFactoryDefaults();

        // Setting an amp limit.
        access.setSmartCurrentLimit(35);
        roller.setSmartCurrentLimit(35);

        // Enabling voltage compansation.
        access.enableVoltageCompensation(12);
        roller.enableVoltageCompensation(12);

        // Inverting the motors as necessary.
        access.setInverted(false);
        roller.setInverted(true);

        // Setting idle modes for each motor.
        access.setIdleMode(IdleMode.kBrake);
        roller.setIdleMode(IdleMode.kCoast);

        // Getting the encoders of each motor.
        accessEncoder = access.getEncoder();
        rollerEncoder = roller.getEncoder();

        // Resetting encoder positions.
        accessEncoder.setPosition(0);
        rollerEncoder.setPosition(0);

        // Setting RPM conversions for each encoder.
        accessEncoder.setPositionConversionFactor(360.0 / IntakeConstants.accessRatio); // To degrees

        // Saving the settings.
        access.burnFlash();
        roller.burnFlash();
    }

    /**
     * Sets the speed of the roller in % speed
     * 
     * @param speed The speed to set the roller to in % speed
     */
    public void setSpeed(double speed) {
        MathUtil.applyDeadband(accessEncoder.getPosition() - 70, 2);

        if (accessEncoder.getPosition() == States.OPEN.value) {
            roller.set(speed * IntakeConstants.maxSpeed);
        }
    }

    public void setAccess(States state) {
        accessPID.setReference(state.value, ControlType.kPosition);
    }
}