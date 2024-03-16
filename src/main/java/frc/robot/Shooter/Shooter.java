package frc.robot.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ShooterConstants;

public class Shooter {
    // Motors
    private TalonFX lShooter = new TalonFX(ShooterConstants.lShooterID);
    private TalonFX rShooter = new TalonFX(ShooterConstants.rShooterID);

    private CANSparkMax feedRoller = new CANSparkMax(ShooterConstants.feedRollerID, MotorType.kBrushless);
    private CANSparkMax align = new CANSparkMax(ShooterConstants.alignID, MotorType.kBrushless);

    private RelativeEncoder alignEncoder = align.getAlternateEncoder(Type.kQuadrature, 8192);

    public Shooter() {
        // Configuring the Krakens
        TalonFXConfiguration lShooterConfig = new TalonFXConfiguration();
        TalonFXConfiguration rShooterConfig = new TalonFXConfiguration();

        lShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        lShooter.getConfigurator().apply(lShooterConfig);
        rShooter.getConfigurator().apply(rShooterConfig);

        // Configuring the SparkMaxes
        feedRoller.restoreFactoryDefaults();
        align.restoreFactoryDefaults();

        feedRoller.setIdleMode(IdleMode.kBrake);
        align.setIdleMode(IdleMode.kCoast);

        feedRoller.burnFlash();
        align.burnFlash();

        // Configuring the align encoder
        alignEncoder.setPositionConversionFactor(ShooterConstants.gearRatio);
        alignEncoder.setVelocityConversionFactor(ShooterConstants.gearRatio / 60);

    }
}