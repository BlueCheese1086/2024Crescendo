package frc.robot.LightArm;

import java.util.Objects;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import Util.DebugPID;
import Util.ThreeState;
import Util.Interfaces.InitializedSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Intake.Intake.IntakeState;


public class LightArm extends SubsystemBase implements InitializedSubsystem {

    private final CANSparkMax armAngle;

    private final RelativeEncoder armEnc;
    private final AbsoluteEncoder armAbs;

    private ArmState state = ArmState.Deployed;

    private final Timer stateBuffer = new Timer();
    private boolean stateOveridden = false;

    private final PIDController anglePID;

    private static LightArm instance;

    public LightArm() {
        armAngle = new CANSparkMax(IntakeConstants.angleID, MotorType.kBrushless); // Replace ID with new arm constants

        armEnc = armAngle.getEncoder();
        armAbs = armAngle.getAbsoluteEncoder(Type.kDutyCycle);

        anglePID = new PIDController(IntakeConstants.kPAngle, IntakeConstants.kIAngle, IntakeConstants.kDAngle); // Replace PID values with new arm constants
    }

    public enum ArmState {
        Stowed(true),
        Deployed(false);

        private final double angleRad;
        
        ArmState(boolean stowed) {
            // If true, arm is in stowed state; if false, arm is in deployed state
            this.angleRad = stowed ? IntakeConstants.STOWED_ANGLE : IntakeConstants.DOWN_ANGLE; // Replace with new Arm angle constants 
        }

        double getArmAngle() {
            return angleRad;
        }

    }

    public static LightArm getInstance() {
        if (Objects.isNull(instance))
            instance = new LightArm();
        return instance;
    }

    public void initialize() {
        armAngle.restoreFactoryDefaults();
        armAngle.setSmartCurrentLimit((int) IntakeConstants.ANGLE_CURRENT_LIMIT); // Replace with new arm constant
        armAngle.setInverted(false);
        armAngle.setIdleMode(IdleMode.kCoast);

        armAbs.setVelocityConversionFactor(IntakeConstants.anglePositionConverstionFactor / 60.0); // Replace all IntakeConstants with new arm constants
        armAbs.setPositionConversionFactor(IntakeConstants.anglePositionConverstionFactor);
        armAbs.setZeroOffset(IntakeConstants.angleOffset);
        armAbs.setInverted(false);

        armEnc.setPositionConversionFactor(2.0 * Math.PI / 54.55);
        armEnc.setVelocityConversionFactor(2.0 * Math.PI / 54.55 / 60.0);
        armEnc.setPosition(armAbs.getPosition());

        armAngle.burnFlash();

        stateBuffer.start();
    }

    public void periodic() {
        SmartDashboard.putNumber("LightArm/Angle/MotorCurrent", armAngle.getOutputCurrent());
        SmartDashboard.putNumber("LightArm/Angle/BusVoltage", armAngle.getBusVoltage());
        SmartDashboard.putNumber("LightArm/Angle/ABSPosition", armAbs.getPosition());
        SmartDashboard.putNumber("LightArm/Angle/Temperature", armAngle.getMotorTemperature());

        SmartDashboard.putNumber("LightArm/DesiredAngle", state.angleRad);
    }

    /**
     * The default method that runs in the command scheduler to ensure everything in the subsystem is behaving as expected
     */
    public void defaultMethod() {
        stateOveridden = false;
        setAnglePosition(state.angleRad);
    }

    /**
     * @return Returns the current angle of the arm in radians
     */
    public double getAngle() {
        return armEnc.getPosition();
    }

    /**
     * @return Returns the desired state of the light arm
     */
    public ArmState getDesiredState() {
        return state;
    }

    /**
     * Sets the goal of the angle position PID to the desired angle in radians
     * 
     * @param angleRads The desired angle in radians
     */
    public void setAnglePosition(double angleRads) {
        double desiredVoltage = anglePID.calculate(armAbs.getPosition(), angleRads) + IntakeConstants.kGAngle /* Replace */ * Math.cos(armAbs.getPosition() - 0.2 /* Maybe replace the 0.2 too since different motor?? */);
        armAngle.setVoltage(desiredVoltage);
    }

    /**
     * Sets the desired state of the LightArm
     * @param state The desired state of the arm
     */
    public void setState(ArmState state) {
        if (stateOveridden) return;
        this.state = state;
    }

    /**
     * Stops the arm angle control motor
     */
    public void stop() {
        armAngle.stopMotor();
    }
}