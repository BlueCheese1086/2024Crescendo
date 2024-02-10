package frc.robot.Climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import Util.IntializedSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase implements IntializedSubsystem {
    
    private final CANSparkMax left, right;
    private final RelativeEncoder leftEnc, rightEnc;
    private final SparkPIDController leftPID, rightPID;

    public Climb() {
        left = new CANSparkMax(ClimbConstants.leftID, MotorType.kBrushless);
        right = new CANSparkMax(ClimbConstants.rightID, MotorType.kBrushless);

        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        left.setSmartCurrentLimit(25);
        right.setSmartCurrentLimit(25);        

        left.setInverted(false);
        right.setInverted(true);

        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        leftEnc = left.getEncoder();
        leftEnc.setPositionConversionFactor(ClimbConstants.climbConversionFactor);
        leftEnc.setVelocityConversionFactor(ClimbConstants.climbConversionFactor / 60.0);

        rightEnc = right.getEncoder();
        rightEnc.setPositionConversionFactor(ClimbConstants.climbConversionFactor);
        rightEnc.setVelocityConversionFactor(ClimbConstants.climbConversionFactor / 60.0);

        leftPID = left.getPIDController();
        leftPID.setP(ClimbConstants.kP);
        leftPID.setI(ClimbConstants.kI);
        leftPID.setD(ClimbConstants.kD);
        leftPID.setFF(ClimbConstants.kFF);

        rightPID = left.getPIDController();
        rightPID.setP(ClimbConstants.kP);
        rightPID.setI(ClimbConstants.kI);
        rightPID.setD(ClimbConstants.kD);
        rightPID.setFF(ClimbConstants.kFF);

        left.burnFlash();
        right.burnFlash();

    }

    public void initialize() {
        leftEnc.setPosition(0.0);
        rightEnc.setPosition(0.0);
    }

    public void setEncToTop() {
        leftEnc.setPosition(ClimbConstants.maxHeight);
        rightEnc.setPosition(ClimbConstants.maxHeight);
    }

    public void periodic() {
        if (leftEnc.getPosition() >= ClimbConstants.maxHeight) left.stopMotor();
        if (rightEnc.getPosition() >= ClimbConstants.maxHeight) right.stopMotor();
        // TODO
        // Telemetry
        // Logging
    }

    /**
     * (0-1) 0 is fully down, 1 is fully up
     * @param leftHeight
     * @param rightHeight
     */
    public void set0to1Position(double leftHeight, double rightHeight) {
        leftPID.setReference(leftHeight * ClimbConstants.climbConversionFactor, ControlType.kPosition);
        rightPID.setReference(rightHeight * ClimbConstants.climbConversionFactor, ControlType.kPosition);
    }

    public void stopMotors() {
        left.stopMotor();
        right.stopMotor();
    }

}
