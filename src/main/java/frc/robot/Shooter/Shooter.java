package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import Util.Interfaces.InitializedSubsystem;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase implements InitializedSubsystem {

    private final CANSparkMax front = new CANSparkMax(ShooterConstants.frontID, MotorType.kBrushless);
    private final CANSparkMax back = new CANSparkMax(ShooterConstants.backID, MotorType.kBrushless);

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

        front.restoreFactoryDefaults();
        back.restoreFactoryDefaults();

        front.setInverted(false);
        back.setInverted(false);

        front.setIdleMode(IdleMode.kCoast);
        back.setIdleMode(IdleMode.kBrake);

        frontEnc = front.getEncoder();
        frontEnc.setVelocityConversionFactor(1.0);

        backEnc = back.getEncoder();
        backEnc.setVelocityConversionFactor(1.0);

        frontPID = front.getPIDController();
        frontPID.setP(ShooterConstants.kP);
        frontPID.setI(ShooterConstants.kI);
        frontPID.setD(ShooterConstants.kD);
        frontPID.setFF(ShooterConstants.kFF);

        backPID = back.getPIDController();
        backPID.setP(ShooterConstants.kP);
        backPID.setI(ShooterConstants.kI);
        backPID.setD(ShooterConstants.kD);
        backPID.setFF(ShooterConstants.kFF);

        front.burnFlash();
        back.burnFlash();

    }

    public void initialize() {
        
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

    public double getTotalCurrent() {
        return front.getOutputCurrent() + back.getOutputCurrent();
    }

    public void setMotorVels(double frontRPM, double backRPM) {
        frontPID.setReference(frontRPM, ControlType.kVelocity);
        backPID.setReference(backRPM, ControlType.kVelocity);
    }

    public void stopMotors() {
        front.stopMotor();
        back.stopMotor();
    }

    public CANSparkMax getFront() {
        return front;
    }

    public CANSparkMax getBack() {
        return back;
    }

}