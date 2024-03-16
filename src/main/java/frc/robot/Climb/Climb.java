package frc.robot.Climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimbConstants;

public class Climb {
    private CANSparkMax lTower = new CANSparkMax(ClimbConstants.lTowerID, MotorType.kBrushless);
    private CANSparkMax rTower = new CANSparkMax(ClimbConstants.rTowerID, MotorType.kBrushless);
}
