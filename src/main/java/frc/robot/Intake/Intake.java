package frc.robot.Intake;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import Util.DebugPID;
import Util.ThreeState;
import Util.Interfaces.InitializedSubsystem;
import Util.Interfaces.PowerManaged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase implements InitializedSubsystem, PowerManaged {

    private final CANSparkMax rollers;
    private final CANSparkMax angle;

    private final DigitalInput shooterNoteDetector;
    private final DigitalInput intakeNoteDetector;

    private final Timer stateBuffer = new Timer();
    private boolean stateOveridden = false;

    private final RelativeEncoder rollersEnc;
    private final RelativeEncoder angleEnc;
    private final AbsoluteEncoder angleAbs;

    private final SparkPIDController rollersPID;
    private final PIDController anglePID;

    private IntakeState state = IntakeState.IdlingUp;

    private static Intake instance;

    public enum IntakeState {
        IntakingDown(false, ThreeState.TRUE),
        IntakingUp(true, ThreeState.TRUE),
        OuttakingUp(true, ThreeState.FALSE),
        OuttakingDown(true, ThreeState.FALSE),
        IdlingUp(true, ThreeState.IDLE),
        IdlingDown(false, ThreeState.IDLE);

        private final double angleRad;
        private final double rollersRpm;
        
        IntakeState(boolean up, ThreeState rollersIn) {
            this.angleRad = up ? IntakeConstants.STOWED_ANGLE : IntakeConstants.DOWN_ANGLE;
            switch (rollersIn) {
                case TRUE:
                    rollersRpm = IntakeConstants.ROLLERS_IN_RPM;
                    break;
                case FALSE:
                    rollersRpm = IntakeConstants.ROLLERS_OUT_RPM;
                    break;
                default:
                    rollersRpm = 0.0;
                    break;
            }
        }

        double getAngle() {
            return angleRad;
        }

        double getRollerState() {
            return rollersRpm;
        }
    }

    public static Intake getInstance() {
        if (Objects.isNull(instance))
            instance = new Intake();
        return instance;
    }

    private Intake() {
        shooterNoteDetector = new DigitalInput(IntakeConstants.shooterNoteDetectorID);
        intakeNoteDetector = new DigitalInput(IntakeConstants.intakeNoteDetectorID);

        rollers = new CANSparkMax(IntakeConstants.rollerID, MotorType.kBrushless);
        angle = new CANSparkMax(IntakeConstants.angleID, MotorType.kBrushless);

        rollersEnc = rollers.getEncoder();
        angleEnc = angle.getEncoder();
        angleAbs = angle.getAbsoluteEncoder(Type.kDutyCycle);

        anglePID = new PIDController(IntakeConstants.kPAngle, IntakeConstants.kIAngle, IntakeConstants.kDAngle);
        rollersPID = rollers.getPIDController();
    }

    public void initialize() {
        rollers.restoreFactoryDefaults();
        angle.restoreFactoryDefaults();

        angle.setSmartCurrentLimit((int) IntakeConstants.ANGLE_CURRENT_LIMIT);
        rollers.setSmartCurrentLimit((int) IntakeConstants.ROLLERS_CURRENT_LIMIT);

        rollers.setInverted(false);
        angle.setInverted(false);

        rollers.setIdleMode(IdleMode.kCoast);
        angle.setIdleMode(IdleMode.kBrake);

        rollersEnc.setVelocityConversionFactor(IntakeConstants.rollersVelocityConversionFactor);

        angleAbs.setVelocityConversionFactor(IntakeConstants.anglePositionConverstionFactor / 60.0);
        angleAbs.setPositionConversionFactor(IntakeConstants.anglePositionConverstionFactor);
        angleAbs.setZeroOffset(IntakeConstants.angleOffset);
        angleAbs.setInverted(false);

        angleEnc.setPositionConversionFactor(2.0 * Math.PI / 54.55);
        angleEnc.setVelocityConversionFactor(2.0 * Math.PI / 54.55 / 60.0);
        angleEnc.setPosition(angleAbs.getPosition());

        rollersPID.setP(IntakeConstants.kPRoller);
        rollersPID.setI(IntakeConstants.kIRoller);
        rollersPID.setD(IntakeConstants.kDRoller);
        rollersPID.setFF(IntakeConstants.kFFRoller);

        rollers.burnFlash();
        angle.burnFlash();

        new DebugPID(rollersPID, "IntakeRollers");
        // new DebugPID(anglePID, "IntakeAngle");

        stateBuffer.start();
    }

    public void periodic() {
        Logger.recordOutput("Intake/Roller/MotorCurrent", rollers.getOutputCurrent());
        Logger.recordOutput("Intake/Roller/BusVoltage", rollers.getBusVoltage());
        Logger.recordOutput("Intake/Roller/Velocity", rollersEnc.getVelocity());
        Logger.recordOutput("Intake/Roller/Temperature", rollers.getMotorTemperature());

        Logger.recordOutput("Intake/Angle/MotorCurrent", angle.getOutputCurrent());
        Logger.recordOutput("Intake/Angle/BusVoltage", angle.getBusVoltage());
        Logger.recordOutput("Intake/Angle/Velocity", angleEnc.getVelocity());
        Logger.recordOutput("Intake/Angle/Temperature", angle.getMotorTemperature());

        SmartDashboard.putNumber("Intake/Roller/MotorCurrent", rollers.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Roller/BusVoltage", rollers.getBusVoltage());
        SmartDashboard.putNumber("Intake/Roller/Velocity", rollersEnc.getVelocity());
        SmartDashboard.putNumber("Intake/Roller/Temperature", rollers.getMotorTemperature());

        SmartDashboard.putNumber("Intake/Angle/MotorCurrent", angle.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Angle/BusVoltage", angle.getBusVoltage());
        SmartDashboard.putNumber("Intake/Angle/ABSPosition", angleAbs.getPosition());
        SmartDashboard.putNumber("Intake/Angle/Temperature", angle.getMotorTemperature());

        SmartDashboard.putBoolean("Intake/ShooterBreakStatus", getShooterSensor());
        SmartDashboard.putBoolean("Intake/IntakeBreakStatus", getIntakeSensor());

        SmartDashboard.putNumber("Intake/DesiredAngle", state.angleRad);
        SmartDashboard.putNumber("Intake/DesiredRPM", state.rollersRpm);
    }

    /**
     * The default method that runs in the command scheduler to ensure everything in the subsystem is behaving as expected
     */
    public void defaultMethod() {
        stateOveridden = false;
        // if (getIntakeSensor() && !getShooterSensor() && state == IntakeState.IntakingDown) {
        //     setState(IntakeState.IntakingUp);
        //     stateOveridden = true;
        // }
        if (getShooterSensor() && getIntakeSensor() && state.rollersRpm > 0.0) {
            setState(IntakeState.IdlingUp);
            stateOveridden = true;
        }

        setAnglePosition(state.angleRad);
        setRollerSpeed(state.rollersRpm);
    }

    /**
     * @return Returns the current angle of the intake in radians
     */
    public double getAngle() {
        return angleEnc.getPosition();
    }

    public double getCurrentLimit() {
        return IntakeConstants.ANGLE_CURRENT_LIMIT + IntakeConstants.ROLLERS_CURRENT_LIMIT;
    }

    /**
     * @return Returns the desired state of the intake
     */
    public IntakeState getDesiredState() {
        return state;
    }

    /**
     * @return Returns the state of the intake beam break sensor
     */
    public boolean getIntakeSensor() {
        return !intakeNoteDetector.get();
    }

    /**
     * @return Returns the state of the shooter beam break sensor
     */
    public boolean getShooterSensor() {
        return !shooterNoteDetector.get();
    }

    public double getTotalCurrent() {
        return angle.getOutputCurrent() + rollers.getOutputCurrent();
    }

    /**
     * Sets the goal of the angle position PID to the desired angle in radians
     * 
     * @param angleRads The desired angle in radians
     */
    public void setAnglePosition(double angleRads) {
        double desiredVoltage = anglePID.calculate(angleAbs.getPosition(), angleRads) + IntakeConstants.kGAngle * Math.cos(angleAbs.getPosition() - 0.2);
        angle.setVoltage(desiredVoltage);
    }

    /**
     * Sets the desired state of the intake
     * @param state The desired state of the intake
     */
    public void setState(IntakeState state) {
        if (stateOveridden) return;
        this.state = state;
    }

    public void setCurrentLimit(int a) {
        rollers.setSmartCurrentLimit(a);
        angle.setSmartCurrentLimit(a);
    }

    /**
     * Sets the goal of the roller velocity PID to the desired rpm
     * 
     * @param rpm The desired rpm
     */
    public void setRollerSpeed(double rpm) {
        // rollers.setVoltage(rollersPID.calculate(rollersEnc.getVelocity(), rpm) +
        // rollersFeedforward.calculate(rpm));
        SmartDashboard.putNumber("Intake/DesiredRPM as Reported", rpm);
        rollersPID.setReference(rpm, ControlType.kVelocity);
    }

    /**
     * Stops all motors in the intake
     */
    public void stop() {
        angle.stopMotor();
        rollers.stopMotor();
    }

    /**
     * Stops the angle control motor
     */
    public void stopAngle() {
        angle.stopMotor();
    }

    /**
     * Stops the roller control motor
     */
    public void stopRollers() {
        rollers.stopMotor();
    }

}