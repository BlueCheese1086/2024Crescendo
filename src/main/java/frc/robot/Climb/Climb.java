package frc.robot.Climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.Objects;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import Util.Interfaces.InitializedSubsystem;
import Util.Interfaces.PowerManaged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase implements PowerManaged, InitializedSubsystem {
    
    private final CANSparkMax left, right;
    private final RelativeEncoder leftEnc, rightEnc;
    private final SparkPIDController leftPID, rightPID;

    private static Climb instance;

    public static Climb getInstance() {
        if (Objects.isNull(instance)) instance = new Climb();
        return instance;
    }

    public Climb() {
        left = new CANSparkMax(ClimbConstants.leftID, MotorType.kBrushless);
        right = new CANSparkMax(ClimbConstants.rightID, MotorType.kBrushless);

        leftEnc = left.getEncoder();
        rightEnc =  right.getEncoder();

        leftPID = left.getPIDController();
        rightPID = right.getPIDController();
    }

    public void initialize() {
        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        left.setSmartCurrentLimit((int) ClimbConstants.CURRENT_LIMIT);
        right.setSmartCurrentLimit((int) ClimbConstants.CURRENT_LIMIT);        

        left.setInverted(false);
        right.setInverted(true);

        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        leftEnc.setPositionConversionFactor(ClimbConstants.climbConversionFactor);
        leftEnc.setVelocityConversionFactor(ClimbConstants.climbConversionFactor / 60.0);
        leftEnc.setPosition(0.0);

        rightEnc.setPositionConversionFactor(ClimbConstants.climbConversionFactor);
        rightEnc.setVelocityConversionFactor(ClimbConstants.climbConversionFactor / 60.0);
        rightEnc.setPosition(0.0);

        leftPID.setP(ClimbConstants.kP);
        leftPID.setI(ClimbConstants.kI);
        leftPID.setD(ClimbConstants.kD);
        leftPID.setFF(ClimbConstants.kFF);

        rightPID.setP(ClimbConstants.kP);
        rightPID.setI(ClimbConstants.kI);
        rightPID.setD(ClimbConstants.kD);
        rightPID.setFF(ClimbConstants.kFF);

        // new DebugPID(leftPID, "Climb/LeftPID");
        // new DebugPID(rightPID, "Climb/RightPID");

        left.burnFlash();
        right.burnFlash();
    }

    public void periodic() {
        if (leftEnc.getPosition() > ClimbConstants.maxHeight) left.stopMotor();
        if (rightEnc.getPosition() > ClimbConstants.maxHeight) right.stopMotor();

        SmartDashboard.putNumber("Climb/LeftPOS", leftEnc.getPosition());
        SmartDashboard.putNumber("Climb/RightPOS", rightEnc.getPosition());

        // TODO
        // Telemetry
        // Logging
    }


    public double getCurrentLimit() {
        return ClimbConstants.CURRENT_LIMIT;
    }

    public double getTotalCurrent() {
        return left.getOutputCurrent() + right.getOutputCurrent();
    }

    public void setCurrentLimit(int a) {
        left.setSmartCurrentLimit(a);
        right.setSmartCurrentLimit(a);
    }

    public void setEncToTop() {
        leftEnc.setPosition(ClimbConstants.maxHeight);
        rightEnc.setPosition(ClimbConstants.maxHeight);
    }

    /**
     * (0-1) 0 is fully down, 1 is fully up
     * @param leftHeight
     * @param rightHeight
     */
    public void setPosition(double leftHeight, double rightHeight) {
        if (leftHeight >= 0.0) leftPID.setReference(leftHeight * ClimbConstants.maxHeight, ControlType.kPosition);
        if (rightHeight >= 0.0) rightPID.setReference(rightHeight * ClimbConstants.maxHeight, ControlType.kPosition);
    }

    /**
     * Stops both climb motors
     */
    public void stopMotors() {
        left.stopMotor();
        right.stopMotor();
    }

}