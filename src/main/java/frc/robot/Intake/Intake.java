package frc.robot.Intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import Util.IntializedSubsystem;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase implements IntializedSubsystem {

    private final CANSparkMax rollers = new CANSparkMax(IntakeConstants.rollerID, MotorType.kBrushless);
    private final CANSparkMax angle = new CANSparkMax(IntakeConstants.angleID, MotorType.kBrushless);

    private final RelativeEncoder rollersEnc;
    private final RelativeEncoder angleEnc;
    private final AbsoluteEncoder angleAbs;

    private final PIDController rollersPID;
    private final PIDController anglePID;
    private final SparkPIDController angleVoltageController;

    private final SimpleMotorFeedforward rollersFeedforward;

    public Intake() {
        rollers.restoreFactoryDefaults();
        angle.restoreFactoryDefaults();

        rollers.setInverted(false);
        angle.setInverted(false);

        rollers.setIdleMode(IdleMode.kCoast);
        angle.setIdleMode(IdleMode.kCoast);

        rollersEnc = rollers.getEncoder();
        rollersEnc.setVelocityConversionFactor(IntakeConstants.rollersVelocityConversionFactor);

        angleAbs = angle.getAbsoluteEncoder(Type.kDutyCycle);
        angleAbs.setVelocityConversionFactor(IntakeConstants.anglePositionConverstionFactor / 60.0);
        angleAbs.setPositionConversionFactor(IntakeConstants.anglePositionConverstionFactor);
        angleAbs.setZeroOffset(IntakeConstants.angleOffset);
        angleAbs.setInverted(false);

        angleEnc = angle.getEncoder();
        angleEnc.setPositionConversionFactor(2.0 * Math.PI / 54.55);
        angleEnc.setVelocityConversionFactor(2.0 * Math.PI / 54.55 / 60.0);
        angleEnc.setPosition(angleAbs.getPosition());

        angleVoltageController = angle.getPIDController();
        angleVoltageController.setP(1.0);

        rollersPID = new PIDController(IntakeConstants.kPRoller, IntakeConstants.kIRoller, IntakeConstants.kDRoller);
        anglePID = new PIDController(IntakeConstants.kPAngle, IntakeConstants.kIAngle, IntakeConstants.kDAngle);

        rollersFeedforward = new SimpleMotorFeedforward(0.0, 0.003, 0.0);

        rollers.burnFlash();
        angle.burnFlash();

        // new DebugPID(rollersPID, "IntakeRollers");
        // new DebugPID(anglePID, "IntakeAngle");
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

        SmartDashboard.putNumber("Intake/AbsAngle", angleAbs.getPosition());
        SmartDashboard.putNumber("Intake/RollerSpeed", rollersEnc.getVelocity());

        SmartDashboard.putNumber("Intake/RelEnc/Measurement", angle.getEncoder().getPosition());

    }

    public RelativeEncoder getRollerEncoder() {
        return rollersEnc;
    }

    public double getAngle() {
        return angleEnc.getPosition();
    }

    public void setAnglePosition(double angleRads) {
        double desiredVoltage = anglePID.calculate(angleAbs.getPosition(), angleRads) + 1.07 * Math.cos(angleAbs.getPosition());// + anglePID.getP() * angleFeedforward.calculate(angleAbs.getPosition(), 0.0);
        SmartDashboard.putNumber("/Intake/DesiredAngle", angleRads);
        SmartDashboard.putNumber("/Intake/DesiredVoltage", desiredVoltage);
        SmartDashboard.putNumber("/Intake/CurrentDraw", angle.getOutputCurrent());
        // angle.setVoltage(desiredVoltage);
        angleVoltageController.setReference(desiredVoltage, ControlType.kVoltage);
    }

    public void setRollerSpeed(double rpm) {
        rollers.setVoltage(rollersPID.calculate(rollersEnc.getVelocity(), rpm) + rollersFeedforward.calculate(rpm));
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
