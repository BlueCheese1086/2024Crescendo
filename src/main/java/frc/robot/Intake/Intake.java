package frc.robot.Intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import Util.DebugPID;
import Util.IntializedSubsystem;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase implements IntializedSubsystem {

    private final CANSparkMax rollers = new CANSparkMax(IntakeConstants.rollerID, MotorType.kBrushless);
    private final CANSparkMax angle = new CANSparkMax(IntakeConstants.angleID, MotorType.kBrushless);

    private final RelativeEncoder rollersEnc;
    private final AbsoluteEncoder angleEnc;

    private final SparkPIDController rollersPID;
    private final SparkPIDController anglePID;

    public Intake() {
        rollers.restoreFactoryDefaults();
        angle.restoreFactoryDefaults();

        rollers.setSmartCurrentLimit(40);
        angle.setSmartCurrentLimit(20);

        rollers.setInverted(true);
        angle.setInverted(true);

        rollers.setIdleMode(IdleMode.kCoast);
        angle.setIdleMode(IdleMode.kCoast);

        rollersEnc = rollers.getEncoder();
        rollersEnc.setVelocityConversionFactor(IntakeConstants.rollersVelocityConversionFactor);

        angleEnc = angle.getAbsoluteEncoder(Type.kDutyCycle);
        angleEnc.setZeroOffset(IntakeConstants.angleOffset);
        angleEnc.setVelocityConversionFactor(IntakeConstants.anglePositionConverstionFactor / 60.0);
        angleEnc.setPositionConversionFactor(IntakeConstants.anglePositionConverstionFactor);

        rollersPID = rollers.getPIDController();
        rollersPID.setP(IntakeConstants.kPRoller);
        rollersPID.setI(IntakeConstants.kIRoller);
        rollersPID.setD(IntakeConstants.kDRoller);
        rollersPID.setFF(IntakeConstants.kFFRoller);

        anglePID = rollers.getPIDController();
        anglePID.setFeedbackDevice(angleEnc);
        anglePID.setP(IntakeConstants.kPAngle);
        anglePID.setI(IntakeConstants.kIAngle);
        anglePID.setD(IntakeConstants.kDAngle);
        anglePID.setFF(IntakeConstants.kFFAngle);

        rollers.burnFlash();
        angle.burnFlash();

        new DebugPID(rollersPID, "IntakeRollers");
        new DebugPID(anglePID, "IntakeAngle");
    }

    public void initialize() {
        
    }

    public void periodic() {
        // TODO
        // Telemetry

        Logger.recordOutput("Intake/Roller/MotorCurrent", rollers.getOutputCurrent());
        Logger.recordOutput("Intake/Roller/BusVoltage", rollers.getBusVoltage());
        Logger.recordOutput("Intake/Roller/Velocity", rollersEnc.getVelocity());
        Logger.recordOutput("Intake/Roller/Temperature", rollers.getMotorTemperature());

        Logger.recordOutput("Intake/Angle/MotorCurrent", angle.getOutputCurrent());
        Logger.recordOutput("Intake/Angle/BusVoltage", angle.getBusVoltage());
        Logger.recordOutput("Intake/Angle/Velocity", angleEnc.getVelocity());
        Logger.recordOutput("Intake/Angle/Temperature", angle.getMotorTemperature());

        SmartDashboard.putNumber("Intake/Angle", angleEnc.getPosition());
        SmartDashboard.putNumber("Intake/RollerSpeed", rollersEnc.getVelocity());
        
    }

    public void setAnglePosition(double angleRads) {
        anglePID.setReference(angleRads, ControlType.kPosition);
    }

    public void setRollerSpeed(double rpm) {
        rollersPID.setReference(rpm, ControlType.kVelocity);
    }

    public void stopRollers() {
        rollers.stopMotor();
    }

    public void stopAngle() {
        angle.stopMotor();
    }

    public void stop() {
        angle.stopMotor();
        rollers.stopMotor();
    }

}
