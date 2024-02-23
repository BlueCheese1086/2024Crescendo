package frc.robot.Intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import Util.DebugPID;
import Util.IntializedSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase implements IntializedSubsystem {

    private final CANSparkMax rollers = new CANSparkMax(IntakeConstants.rollerID, MotorType.kBrushless);
    private final CANSparkMax angle = new CANSparkMax(IntakeConstants.angleID, MotorType.kBrushless);

    private final RelativeEncoder rollersEnc;
    private final AbsoluteEncoder angleEnc;

    private final PIDController rollersPID;
    private final PIDController anglePID;
    private final ArmFeedforward angleFeedforward;
    private final SimpleMotorFeedforward rollersFeedforward;

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
        angleEnc.setVelocityConversionFactor(IntakeConstants.anglePositionConverstionFactor / 60.0);
        angleEnc.setPositionConversionFactor(IntakeConstants.anglePositionConverstionFactor);
        angleEnc.setZeroOffset(IntakeConstants.angleOffset);
        angleEnc.setInverted(true);


        rollersPID = new PIDController(IntakeConstants.kPRoller, IntakeConstants.kIRoller, IntakeConstants.kDRoller);

        anglePID = new PIDController(IntakeConstants.kPAngle, IntakeConstants.kIAngle, IntakeConstants.kDAngle);

        angleFeedforward = new ArmFeedforward(0.05, 1.07, 0.51);
        rollersFeedforward = new SimpleMotorFeedforward(1.0, 0.26, 2.22);

        rollers.burnFlash();
        angle.burnFlash();

        new DebugPID(rollersPID, "IntakeRollers");
        new DebugPID(anglePID, "IntakeAngle");
    }

    public void initialize() {}

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

    public RelativeEncoder getRollerEncoder() {
        return rollersEnc;
    }

    public double getAngle() {
        return angleEnc.getPosition();
    }

    public void setAnglePosition(double angleRads) {
        angle.setVoltage(anglePID.calculate(angleEnc.getPosition(), angleRads) + angleFeedforward.calculate(angleEnc.getPosition(), angleRads));
        // anglePID.setReference(angleRads, ControlType.kPosition);
    }

    public void setRollerSpeed(double rpm) {
        rollers.setVoltage(rollersPID.calculate(rollersEnc.getVelocity(), rpm) + rollersFeedforward.calculate(rollersEnc.getVelocity()));
        // rollersPID.setReference(rpm, ControlType.kVelocity);
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
