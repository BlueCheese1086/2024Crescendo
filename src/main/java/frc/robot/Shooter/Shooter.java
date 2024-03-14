package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import Util.Interfaces.InitializedSubsystem;
import Util.Interfaces.PowerManaged;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase implements InitializedSubsystem, PowerManaged {

    private final CANSparkMax front;
    private final CANSparkMax back;

    private final RelativeEncoder frontEnc;
    private final RelativeEncoder backEnc;

    private final SparkPIDController frontPID;
    private final SparkPIDController backPID;

    private static Shooter instance;

    public static Shooter getInstance() {
        if (Objects.isNull(instance)) instance = new Shooter();
        return instance;
    }
    
    private Shooter() {
        front = new CANSparkMax(ShooterConstants.frontID, MotorType.kBrushless);
        back = new CANSparkMax(ShooterConstants.backID, MotorType.kBrushless);

        frontEnc = front.getEncoder();
        backEnc = back.getEncoder();

        frontPID = front.getPIDController();
        backPID = back.getPIDController();
    }

    public void initialize() {
        front.restoreFactoryDefaults();
        back.restoreFactoryDefaults();

        front.setInverted(false);
        back.setInverted(false);

        front.setIdleMode(IdleMode.kCoast);
        back.setIdleMode(IdleMode.kBrake);

        frontEnc.setVelocityConversionFactor(1.0);
        backEnc.setVelocityConversionFactor(1.0);

        frontPID.setP(ShooterConstants.kP);
        frontPID.setI(ShooterConstants.kI);
        frontPID.setD(ShooterConstants.kD);
        frontPID.setFF(ShooterConstants.kFF);

        backPID.setP(ShooterConstants.kP);
        backPID.setI(ShooterConstants.kI);
        backPID.setD(ShooterConstants.kD);
        backPID.setFF(ShooterConstants.kFF);

        front.burnFlash();
        back.burnFlash();
    }

    public void periodic() {
        // TODO
        // Telemetry

        Logger.recordOutput("Shooter/Front/MotorCurrent", front.getOutputCurrent());
        Logger.recordOutput("Shooter/Front/BusVoltage", front.getBusVoltage());
        Logger.recordOutput("Shooter/Front/Velocity", frontEnc.getVelocity());
        Logger.recordOutput("Shooter/Front/Temperature", front.getMotorTemperature());

        Logger.recordOutput("Shooter/Back/MotorCurrent", back.getOutputCurrent());
        Logger.recordOutput("Shooter/Back/BusVoltage", back.getBusVoltage());
        Logger.recordOutput("Shooter/Back/Velocity", backEnc.getVelocity());
        Logger.recordOutput("Shooter/Back/Temperature", back.getMotorTemperature());
    }

    public double getCurrentLimit() {
        return 40.0;
    }

    public double getFrontRPM() {
        return frontEnc.getVelocity();
    }

    public double getTotalCurrent() {
        return front.getOutputCurrent() + back.getOutputCurrent();
    }

    public void setCurrentLimit(int a) {}

    public void stopMotors() {
        front.stopMotor();
        back.stopMotor();
    }

    public void setMotorVels(double frontRPM, double backRPM) {
        frontPID.setReference(frontRPM, ControlType.kVelocity);
        backPID.setReference(backRPM, ControlType.kVelocity);
    }

}