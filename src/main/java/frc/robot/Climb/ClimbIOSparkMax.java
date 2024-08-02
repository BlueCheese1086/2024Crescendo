package frc.robot.Climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Climb.ClimberIOInputsAutoLogged;

public class ClimbIOSparkMax implements ClimbIO {
    private final CANSparkMax motor = new CANSparkMax(RobotMap.Climber.climber, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    
    private final SparkPIDController pid = motor.getPIDController();

    public ClimbIOSparkMax() {

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setCANTimeout(250);
        motor.setInverted(false);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(30);

        encoder.setPositionConversionFactor(ClimberConstants.encoderConversion);
        encoder.setVelocityConversionFactor(ClimberConstants.encoderConversion / 60);

        pid.setP(ClimberConstants.kPReal);
        pid.setI(ClimberConstants.kIReal);
        pid.setD(ClimberConstants.kDReal);
        pid.setFF(ClimberConstants.kFFReal);
        pid.setOutputRange(-1, 1);

        motor.burnFlash();
    }

    @Override
    public void processInputs(final ClimberIOInputsAutoLogged inputs) {
        inputs.climberPositionMeters = encoder.getPosition();
        inputs.climberVelocityMetersPerSecond = encoder.getVelocity();
        inputs.climberAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput(); 
        inputs.climberCurrentAmps = motor.getOutputCurrent();
        inputs.climberTempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setTargetMeters(final double meters) {
        pid.setReference(meters, ControlType.kPosition);
    }

    @Override
    public void setVoltage(final double volts) {
        motor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override
    public void setOpenLoopDutyCycle(final double dutyCycle) {
        motor.set(MathUtil.clamp(dutyCycle, -1, 1));
    }

    @Override
    public void resetEncoder(final double position) {
        encoder.setPosition(position);
    }

	@Override
	public void stop() {
		motor.setVoltage(0);
	}

	@Override
	public void setPID(double kP, double kI, double kD) {
		pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
	}

	@Override
	public void setSimpleFF(double kFF) {
		pid.setFF(kFF);
	}

	@Override
	public void resetEncoder() {
		resetEncoder(0.0);
	}

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {
        // TODO Auto-generated method stub
    }
}