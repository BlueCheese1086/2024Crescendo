package Util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMax extends SubsystemBase {

    private final CANSparkMax sparkMax;

    public SparkMax(int id, MotorType type) {
        sparkMax = new CANSparkMax(id, type);
    }

    public CANSparkMax getSparkMax() {
        return  sparkMax;
    }

}
