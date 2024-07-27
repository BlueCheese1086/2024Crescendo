package frc.robot.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    // Motor
    private CANSparkMax rollerMotor;
    private CANSparkMax accessMotor;

    // Access Encoder
    private RelativeEncoder accessEncoder;
    private AbsoluteEncoder accessABSEncoder;

    // Access PIDController
    private SparkPIDController accessPID;

    // A common instance of the intake subsystem.
    private static Intake instance;

    /**
     * The different states of the intake.
     * 
     * OPEN is where the intake is down and the robot can run the intake.
     * CLOSED is where the intake is up and the robot cannot run the intake.
     */
    public enum States {
        DOWN(1.9),
        UP(0.1);

        public final double value;

        States(double angle){
            this.value = angle;
        }
    }

    public Intake() {
        // Initializing motors
        rollerMotor = new CANSparkMax(IntakeConstants.rollerID, MotorType.kBrushless);
        accessMotor = new CANSparkMax(IntakeConstants.accessID, MotorType.kBrushless);

        rollerMotor.restoreFactoryDefaults();
        accessMotor.restoreFactoryDefaults();

        // Initializing encoder
        accessEncoder = accessMotor.getEncoder();
        accessABSEncoder = accessMotor.getAbsoluteEncoder(Type.kDutyCycle);

        accessABSEncoder.setInverted(true);

        // Setting position conversion factor
        accessEncoder.setPositionConversionFactor(IntakeConstants.anglePosConversionFactor);
        accessABSEncoder.setPositionConversionFactor(IntakeConstants.absPosConversionFactor);

        accessEncoder.setPosition(accessABSEncoder.getPosition());
        if (accessEncoder.getPosition() > 2.7) accessEncoder.setPosition(0);

        // Initializing PID controller
        accessPID = accessMotor.getPIDController();

        // Setting PID values
        accessPID.setP(IntakeConstants.accessP);
        accessPID.setI(IntakeConstants.accessI);
        accessPID.setD(IntakeConstants.accessD);
        accessPID.setFF(IntakeConstants.accessFF);

        accessPID.setFeedbackDevice(accessEncoder);

        rollerMotor.burnFlash();
        accessMotor.burnFlash();
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
        SmartDashboard.putNumber("/Intake/ABS_Encoder_Pos", accessABSEncoder.getPosition());
        SmartDashboard.putNumber("/Intake/REL_Encoder_Pos", accessEncoder.getPosition());
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
    public void setAngle(double angle) {
        SmartDashboard.putNumber("Intake/angle_setpoint", angle);
        accessPID.setReference(angle, ControlType.kPosition);
    }
}