package frc.robot.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
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
        OPEN(0),
        CLOSED(1.7);

        public final double value;

        States(double angle){
            this.value = angle;
        }
    }

    public Intake() {
        // Initializing motors
        rollerMotor = new CANSparkMax(IntakeConstants.rollerID, MotorType.kBrushless);
        accessMotor = new CANSparkMax(IntakeConstants.accessID, MotorType.kBrushless);

        // Initializing encoder
        accessEncoder = accessMotor.getEncoder();
        accessABSEncoder = accessMotor.getAbsoluteEncoder();

        // Setting position conversion factor
        accessEncoder.setPositionConversionFactor(IntakeConstants.anglePosConversionFactor);
        accessABSEncoder.setPositionConversionFactor(IntakeConstants.absPosConversionFactor);

        // Initializing PID controller
        accessPID = accessMotor.getPIDController();

        // Setting PID values
        accessPID.setP(IntakeConstants.accessP);
        accessPID.setI(IntakeConstants.accessI);
        accessPID.setD(IntakeConstants.accessD);
        accessPID.setFF(IntakeConstants.accessFF);

        accessEncoder.setPosition(accessABSEncoder.getPosition());
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
        SmartDashboard.putNumber("Encoder Position", accessABSEncoder.getPosition());
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
        SmartDashboard.putNumber("/Intake/Angle", angle);
        accessPID.setReference(angle, ControlType.kPosition);
    }
}