package frc.robot.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Intake extends SubsystemBase {
    // Motor
    private CANSparkMax rollerMotor;
    private CANSparkMax accessMotor;

    // Access Encoder
    private RelativeEncoder accessEncoder;
    private AbsoluteEncoder accessABSEncoder;

    // Access PIDController
    private SparkPIDController accessPID;
    private ArmFeedforward accessFeedforward;

    // A common instance of the intake subsystem.
    private static Intake instance;

    /**
     * The different states of the intake.
     * 
     * OPEN is where the intake is down and the robot can run the intake.
     * CLOSED is where the intake is up and the robot cannot run the intake.
     */
    public enum States {
        UP(Rotation2d.fromRadians(0.25)),
        DOWN(Rotation2d.fromRadians(2.1));

        public final Rotation2d value;

        States(Rotation2d angle){
            this.value = angle;
        }
    }

    public Intake() {
        // Initializing motors
        rollerMotor = new CANSparkMax(IntakeConstants.rollerID, MotorType.kBrushless);
        accessMotor = new CANSparkMax(IntakeConstants.accessID, MotorType.kBrushless);

        while (rollerMotor.restoreFactoryDefaults() != REVLibError.kOk) {}
        while (accessMotor.restoreFactoryDefaults() != REVLibError.kOk) {}

        // Initializing encoder
        accessEncoder = accessMotor.getEncoder();
        accessABSEncoder = accessMotor.getAbsoluteEncoder(Type.kDutyCycle);

        while (accessABSEncoder.setInverted(true) != REVLibError.kOk) {}

        // Setting position conversion factor
        while (accessEncoder.setPositionConversionFactor(IntakeConstants.anglePosConversionFactor) != REVLibError.kOk) {}
        while (accessABSEncoder.setPositionConversionFactor(IntakeConstants.absPosConversionFactor) != REVLibError.kOk) {}

        while (accessEncoder.setPosition(accessABSEncoder.getPosition()) != REVLibError.kOk) {}
        // if (accessEncoder.getPosition() > 2.7) accessEncoder.setPosition(0);

        // Initializing PID controller
        accessPID = accessMotor.getPIDController();

        // Setting PID values
        while (accessPID.setP(IntakeConstants.accessP) != REVLibError.kOk) {}
        while (accessPID.setI(IntakeConstants.accessI) != REVLibError.kOk) {}
        while (accessPID.setD(IntakeConstants.accessD) != REVLibError.kOk) {}

        // Getting Feedforward
        accessFeedforward = new ArmFeedforward(ShooterConstants.kS, ShooterConstants.kG, ShooterConstants.kV, ShooterConstants.kA);

        while (accessPID.setFeedbackDevice(accessEncoder) != REVLibError.kOk) {}

        while (rollerMotor.burnFlash() != REVLibError.kOk) {}
        while (accessMotor.burnFlash() != REVLibError.kOk) {}
    }

    /**
     * This function gets a common instance of the intake subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Intake subsystem.
     */
    public static Intake getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (instance == null) instance = new Intake();

        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("/Intake/ABS_Encoder_Pos", accessABSEncoder.getPosition() * 180 / Math.PI);
        SmartDashboard.putNumber("/Intake/REL_Encoder_Pos", accessEncoder.getPosition() * 180 / Math.PI);
    }

    /**
     * Sets the speed of the roller motor.
     * 
     * @param speed The percent speed of the roller motor.
     */
    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }

    /**
     * Sets the angle of the access motor.
     * 
     * @param angle The angle that the intake should be set to.
     */
    public void setAngle(Rotation2d angle) {
        SmartDashboard.putNumber("Intake/angle_setpoint", angle.getDegrees());
        while (accessPID.setReference(angle.getRadians(), ControlType.kPosition, 9, accessFeedforward.calculate(angle.getRadians(), accessEncoder.getVelocity()), ArbFFUnits.kVoltage) != REVLibError.kOk) {}
    }
}