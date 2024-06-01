package frc.robot.Shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Pivot extends SubsystemBase {
    // Motor controllers
    private CANSparkMax align = new CANSparkMax(ShooterConstants.alignID, MotorType.kBrushless);

    // Encoders
    private RelativeEncoder alignEncoder;

    // PID Controllers
    private SparkPIDController alignPID;

    // A common instance of the Pivot class
    private static Pivot instance;

    // Preset positions for the shooter
    public static class Positions {
        public Rotation2d ORIGIN = new Rotation2d(0);
        public Rotation2d SPEAKER = new Rotation2d(5 / 9 * Math.PI);
        public Rotation2d AMP = new Rotation2d(Math.PI);
    }   

    // Enum for subsystem states
    public enum State {
        STOWED,
        STATIC_KITBOT,
        ACTIVE_HOMING,
        PREMTIVE_HOMING
    }

    /**
     * Gets an instance of the pivot class that can be used anywhere.
     * If no instance has been created, one is instantiated and assigned to the instance variable.
     * 
     * @return An instance of the Pivot class.
     */
    public static Pivot getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (instance == null) instance = new Pivot();

        return instance;
    }

    public Pivot() {
        // Resetting the settings of the sparkmax
        align.restoreFactoryDefaults();

        // Setting the idle mode of the sparkmax
        align.setIdleMode(IdleMode.kCoast);

        // Getting the align motor's encoder
        alignEncoder = align.getAlternateEncoder(Type.kQuadrature, 8192);

        // Setting the conversion factors for the align encoder
        alignEncoder.setPositionConversionFactor(ShooterConstants.alignPosConversionFactor);

        // Getting the align motor's PID
        alignPID = align.getPIDController();

        // Setting PIDFF values
        alignPID.setP(ShooterConstants.kP);
        alignPID.setI(ShooterConstants.kI);
        alignPID.setD(ShooterConstants.kD);
        alignPID.setFF(ShooterConstants.kFF);

        // Saving the settings of the sparkmax
        align.burnFlash();
    }

    /** Resets the encoder of the align motor. */
    public void resetEncoders() {
        alignEncoder.setPosition(0);
    }

    /**
     * Gets the angle of the shooter mechanism.
     * 
     * @return the angle 
     */
    public Rotation2d getAngle() {
        return new Rotation2d(alignEncoder.getPosition());
    }

    /**
     * Sets the angle of the shooter mechanism.
     * 
     * @param angle The angle to go to as a Rotation2d.
     */
    public void setAngle(Rotation2d angle) {
        alignPID.setReference(angle.getRadians(), ControlType.kPosition);
    }
}
