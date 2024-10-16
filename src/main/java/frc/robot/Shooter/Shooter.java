package frc.robot.Shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    /**
     * The different positions of the shooter.
     * 
     * UP is where the shooter is up.
     * DOWN is where the shooter is down.
     */
    public enum Positions {
        UP(Rotation2d.fromRadians(0.01)),
        DOWN(Rotation2d.fromRadians(0.01 + Math.PI / 4));

        public final Rotation2d value;

        Positions(Rotation2d angle){
            this.value = angle;
        }
    }

    // Motors
    private CANSparkMax lLauncher = new CANSparkMax(ShooterConstants.lLauncherID, MotorType.kBrushless);
    private CANSparkMax rLauncher = new CANSparkMax(ShooterConstants.rLauncherID, MotorType.kBrushless);
    private CANSparkMax feed = new CANSparkMax(ShooterConstants.feedID, MotorType.kBrushless);
    private CANSparkMax pivot = new CANSparkMax(ShooterConstants.pivotID, MotorType.kBrushless);

    // Encoders
    private SparkAbsoluteEncoder pivotEncoder;

    // PID Controllers
    private SparkPIDController pivotController;
    private ArmFeedforward pivotFeedforward;

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
        while (lLauncher.restoreFactoryDefaults() != REVLibError.kOk) {}
        while (rLauncher.restoreFactoryDefaults() != REVLibError.kOk) {}
        while (feed.restoreFactoryDefaults() != REVLibError.kOk) {}
        while (pivot.restoreFactoryDefaults() != REVLibError.kOk) {}
        System.out.println("Reset factory defaults");

        // Setting the idle mode of the sparkmaxes
        while (lLauncher.setIdleMode(IdleMode.kBrake) != REVLibError.kOk) {}
        while (rLauncher.setIdleMode(IdleMode.kBrake) != REVLibError.kOk) {}
        while (feed.setIdleMode(IdleMode.kBrake) != REVLibError.kOk) {}
        while (pivot.setIdleMode(IdleMode.kCoast) != REVLibError.kOk) {}
        System.out.println("Configured idle modes");

        // Making the rShooter motor follow the lShooter motor
        lLauncher.setInverted(true);
        feed.setInverted(true);
        pivot.setInverted(true);
        System.out.println("Inverted motors");

        while (rLauncher.follow(lLauncher, true) != REVLibError.kOk) {}
        System.out.println("Made rLauncher follow lLauncher");

        // Getting the align motor's encoder
        pivotEncoder = pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        // Setting the conversion factors for the align encoder
        while (pivotEncoder.setPositionConversionFactor(ShooterConstants.alignPosConversionFactor) != REVLibError.kOk) {}
        while (pivotEncoder.setInverted(true) != REVLibError.kOk) {}
        System.out.println("Configured pivot encoders");
        // pivotEncoder.setZeroOffset(0.5);

        // Setting the pivot feed forward
        pivotFeedforward = new ArmFeedforward(ShooterConstants.kS, ShooterConstants.kG, ShooterConstants.kV, ShooterConstants.kA);

        // Getting the align motor's PID
        pivotController = pivot.getPIDController();

        // Setting PIDFF values
        while (pivotController.setP(ShooterConstants.kP) != REVLibError.kOk) {}
        while (pivotController.setI(ShooterConstants.kI) != REVLibError.kOk) {}
        while (pivotController.setD(ShooterConstants.kD) != REVLibError.kOk) {}
        System.out.println("Configured PID");

        while (pivotController.setFeedbackDevice(pivotEncoder) != REVLibError.kOk) {}
        System.out.println("Changed pivot PID encoder");

        // Saving the settings of the sparkmaxes
        while (lLauncher.burnFlash() != REVLibError.kOk) {}
        while (rLauncher.burnFlash() != REVLibError.kOk) {}
        while (feed.burnFlash() != REVLibError.kOk) {}
        while (pivot.burnFlash() != REVLibError.kOk) {}
        System.out.println("Saved REV configs");
    }

    /**
     * Gets the angle of the shooter mechanism.
     * 
     * @return the angle 
     */
    public Rotation2d getAngle() {
        return new Rotation2d(pivotEncoder.getPosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("/Shooter/Pivot Angle", pivotEncoder.getPosition());
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
        SmartDashboard.putNumber("/Shooter/Expected_Pos", angle.getRadians());
        while (pivotController.setReference(angle.getRadians(), ControlType.kPosition, 0, pivotFeedforward.calculate(pivotEncoder.getPosition(), pivotEncoder.getVelocity()), ArbFFUnits.kVoltage) != REVLibError.kOk) {}
    }
}