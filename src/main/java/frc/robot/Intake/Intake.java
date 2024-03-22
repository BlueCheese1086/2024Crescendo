package frc.robot.Intake;

import java.util.Objects;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class Intake {
    // Motor
    private CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.rollerID, MotorType.kBrushless);
    private CANSparkMax accessMotor = new CANSparkMax(IntakeConstants.accessID, MotorType.kBrushless);

    // A common instance of the intake subsystem.
    private static Intake instance;

    /**
     * This function gets a common instance of the intake subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Intake subsystem.
     */
    public static Intake getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (Objects.isNull(instance)) instance = new Intake();

        return instance;
    }
}