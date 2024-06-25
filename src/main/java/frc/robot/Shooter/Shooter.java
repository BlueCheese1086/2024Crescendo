package frc.robot.Shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    // Preset positions for the shooter
    public static class Positions {
        public Rotation2d ORIGIN = new Rotation2d(0);
        public Rotation2d SPEAKER = new Rotation2d(5 / 9 * Math.PI);
        public Rotation2d AMP = new Rotation2d(Math.PI);
    }

    // Motors
    private CANSparkMax lLauncher = new CANSparkMax(ShooterConstants.lLauncherID, MotorType.kBrushless);
    private CANSparkMax rLauncher = new CANSparkMax(ShooterConstants.rLauncherID, MotorType.kBrushless);
    private CANSparkMax feed = new CANSparkMax(ShooterConstants.feedID, MotorType.kBrushless);
    private CANSparkMax pivot = new CANSparkMax(ShooterConstants.pivotID, MotorType.kBrushless);

    // Encoders
    private RelativeEncoder pivotEncoder;
    private AbsoluteEncoder absoluteEncoder;

    // PID Controllers
    private SparkPIDController pivotController;

    // A common instance of the shooter subsystem.
    private static Shooter instance;

    /**
     * This function gets a common instance of the shooter subsystem that anyone can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Shooter subsystem.
     */
    public static Shooter getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (instance == null) instance = new Shooter();

        return instance;
    }

    public Shooter() {
        // Resetting the settings of the sparkmaxes
        lLauncher.restoreFactoryDefaults();
        rLauncher.restoreFactoryDefaults();
        feed.restoreFactoryDefaults();
        pivot.restoreFactoryDefaults();

        // Setting the idle mode of the sparkmaxes
        lLauncher.setIdleMode(IdleMode.kBrake);
        rLauncher.setIdleMode(IdleMode.kBrake);
        feed.setIdleMode(IdleMode.kBrake);
        pivot.setIdleMode(IdleMode.kCoast);

        // Making the rShooter motor follow the lShooter motor
        lLauncher.setInverted(true);
        rLauncher.follow(lLauncher, true);
        feed.setInverted(true);
        pivot.setInverted(false);

        // Getting the align motor's encoder
        pivotEncoder = pivot.getAlternateEncoder(Type.kQuadrature, 8192);

        // Setting the conversion factors for the align encoder
        pivotEncoder.setPositionConversionFactor(ShooterConstants.alignPosConversionFactor);

        // Getting the align motor's PID
        pivotController = pivot.getPIDController();

        // Setting PIDFF values
        pivotController.setP(ShooterConstants.kP);
        pivotController.setI(ShooterConstants.kI);
        pivotController.setD(ShooterConstants.kD);
        pivotController.setFF(ShooterConstants.kFF);

        // Saving the settings of the sparkmaxes
        lLauncher.burnFlash();
        rLauncher.burnFlash();
        feed.burnFlash();
        pivot.burnFlash();
    }

    /**
     * Gets the angle of the shooter mechanism.
     * 
     * @return the angle 
     */
    public Rotation2d getAngle() {
        return new Rotation2d(pivotEncoder.getPosition());
    }

    /**
     * Sets the speeds of the left and right shooter motors.
     * 
     * @param speed The speed of the motors in duty cycle output.
     */
    public void runLauncher(double speed) {
        lLauncher.set(speed * ShooterConstants.maxShootSpeed);
    }

    /**
     * Sets the speed of the feed rollers.
     * 
     * @param speed The speed of the motor in duty cycle output.
     */
    public void runFeed(double speed) {
        feed.set(speed * ShooterConstants.maxFeedSpeed);
    }

    /**
     * Sets the angle of the shooter mechanism.
     * 
     * @param angle The angle to go to as a Rotation2d.
     */
    public void setAngle(Rotation2d angle) {
        if (angle.getDegrees() > 75) return;
        if (angle.getDegrees() < 0) return;

        pivotController.setReference(angle.getRadians(), ControlType.kPosition);
    }
}