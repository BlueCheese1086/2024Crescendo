package frc.robot.Shooter;

import java.util.Objects;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants.ShooterConstants;

public class Shooter {
    // Small set of recommended positions.
    public static class Positions {
        public Rotation2d ORIGIN = new Rotation2d(0);
        public Rotation2d SPEAKER = new Rotation2d(5 / 9 * Math.PI);
        public Rotation2d AMP = new Rotation2d(Math.PI);
    }

    // Motors
    private TalonFX lShooter = new TalonFX(ShooterConstants.lShooterID);
    private TalonFX rShooter = new TalonFX(ShooterConstants.rShooterID);

    private CANSparkMax feedRoller = new CANSparkMax(ShooterConstants.feedRollerID, MotorType.kBrushless);
    private CANSparkMax align = new CANSparkMax(ShooterConstants.alignID, MotorType.kBrushless);

    // Encoders
    private RelativeEncoder alignEncoder;

    // PID Controllers
    private SparkPIDController alignPID;

    // A common instance of the shooter subsystem.
    private static Shooter instance;

    public Shooter() {
        // Creating the configuration objects for the talons
        TalonFXConfiguration lShooterConfig = new TalonFXConfiguration();
        TalonFXConfiguration rShooterConfig = new TalonFXConfiguration();

        // Setting the idle mode of the talons
        lShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Updating the settings of the talons
        lShooter.getConfigurator().apply(lShooterConfig);
        rShooter.getConfigurator().apply(rShooterConfig);

        // Making the rShooter motor follow the lShooter motor
        rShooter.setControl(new Follower(lShooter.getDeviceID(), true));

        // Resetting the settings of the sparkmaxes
        feedRoller.restoreFactoryDefaults();
        align.restoreFactoryDefaults();

        // Setting the idle mode of the sparkmaxes
        feedRoller.setIdleMode(IdleMode.kBrake);
        align.setIdleMode(IdleMode.kCoast);

        // Saving the settings of the sparkmaxes
        feedRoller.burnFlash();
        align.burnFlash();

        // Configuring the align encoder
        alignEncoder = align.getAlternateEncoder(Type.kQuadrature, 8192);

        // Setting the conversion factors for the align encoder
        alignEncoder.setPositionConversionFactor(ShooterConstants.gearRatio * 2 * Math.PI);
        alignEncoder.setVelocityConversionFactor(ShooterConstants.gearRatio * 2 * Math.PI / 60);

        // Configuring the align PID
        alignPID = align.getPIDController();

        // Setting PIDFF values
        alignPID.setP(ShooterConstants.kP);
        alignPID.setI(ShooterConstants.kI);
        alignPID.setD(ShooterConstants.kD);
        alignPID.setFF(ShooterConstants.kFF);
    }

    /**
     * This function gets a common instance of the shooter subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Shooter subsystem.
     */
    public static Shooter getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (Objects.isNull(instance)) instance = new Shooter();

        return instance;
    }

    /** Resets the encoder of the align motor. */
    public void resetEncoders() {
        alignEncoder.setPosition(0);
    }

    /**
     * Sets the speeds of the left and right shooter motors.
     * 
     * @param speed The speed of the motors in duty cycle output.
     */
    public void runLauncher(double speed) {
        lShooter.set(speed * ShooterConstants.maxShootSpeed);
    }

    /**
     * Sets the speed of the feed rollers.
     * 
     * @param speed The speed of the motor in duty cycle output.
     */
    public void runFeed(double speed) {
        feedRoller.set(speed * ShooterConstants.maxFeedSpeed);
    }

    /**
     * Finds the angle that the system should be set to based upon camera inputs.
     * 
     * @return The angle the system should be set to.
     */
    public Rotation2d calculateAngle() {
        return new Rotation2d();
    }

    /**
     * Returns the angle of the Shooter.
     * 
     * @return The angle of the Shooter.
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