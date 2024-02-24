package frc.robot.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    // Enum for setpoints
    public enum States {
        AMP(0),
        OPEN(0),
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
        access.setSmartCurrentLimit(15);
        roller.setSmartCurrentLimit(15);

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

    /** Runs every 20 ms */
    public void periodic() {
        SmartDashboard.putNumber("/Intake/Access/Position", accessEncoder.getPosition());
        SmartDashboard.putNumber("/Intake/Access/Speed", access.get());
        SmartDashboard.putNumber("/Intake/Roller/Speed", roller.get());
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

    /**
     * Sets the state of the intake (open, closed, amp)
     * @param state
     */
    public void setAccess(States state) {
        // For now, open moves up, closed moves down, amp is not moving.
        switch (state) {
            case OPEN:
                access.set(0.2);
            case CLOSED:
                access.set(-0.2);
            case AMP:
                access.set(0);
        }
        // Want this to work with setpoints
        // accessPID.setReference(state.value, ControlType.kPosition);
    }
}