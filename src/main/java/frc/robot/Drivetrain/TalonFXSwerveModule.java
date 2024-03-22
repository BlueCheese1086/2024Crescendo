package frc.robot.Drivetrain;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class TalonFXSwerveModule extends SubsystemBase {
    // Motors
    private TalonFX drive;
    private TalonFX turn;

    // Sensors
    private CANcoder cancoder;
    private double offset;

    // Module Vars
    private SwerveModuleState state;
    private SwerveModulePosition position;

    // Closed Loop Control
    private SimpleMotorFeedforward feedForward;
    private VelocityVoltage driveVolts;
    private PositionVoltage turnVolts;

    public TalonFXSwerveModule(int driveID, int turnID, int cancoderID, double offset) {
        // Initializing the motors
        drive = new TalonFX(driveID);
        turn = new TalonFX(turnID);

        // Creating the config objects for each motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();

        // Setting the neutral mode for the motors
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Setting PID values for each motor
        driveConfig.Slot0.kP = DriveConstants.driveP;
        driveConfig.Slot0.kI = DriveConstants.driveI;
        driveConfig.Slot0.kD = DriveConstants.driveD;

        turnConfig.Slot0.kP = DriveConstants.turnP;
        turnConfig.Slot0.kI = DriveConstants.turnI;
        turnConfig.Slot0.kD = DriveConstants.turnD;

        // Initializing the cancoder (may be able to remove due to how talons treat external encoders)
        cancoder = new CANcoder(cancoderID);
        
        // Linking the cancoder to the turn motor
        turnConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        // Initializing Closed-loop Control vars
        feedForward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);
        driveVolts = new VelocityVoltage(0).withSlot(0);
        turnVolts = new PositionVoltage(0).withSlot(0);

        // Updating the configs for each motor
        drive.getConfigurator().apply(driveConfig);
        turn.getConfigurator().apply(turnConfig);
    }

    /** Runs every tick that the subsystem exists. */
    public void periodic() {
        this.state = new SwerveModuleState(getVelocity(), getAngle());
        this.position = new SwerveModulePosition(getDistance(), getAngle());
    }

    /**
     * Gets the acceleration of the swerve module.
     * 
     * @return The acceleration of the swerve module as a double.
     */
    public double getAcceleration() {
        // Gettings Rotations/Second^2
        double rps2 = drive.getAcceleration().getValue();
        
        // Converting Rotations/Second^2 to Meters/Second^2
        return rps2 * DriveConstants.driveRatio * DriveConstants.wheelCircumference;
    }

    /**
     * Gets the angle of the serve module.
     * 
     * @return The angle of the swerve module as a Rotation2d.
     */
    public Rotation2d getAngle() {
        // Getting Rotations
        double rotations = turn.getPosition().getValue() - offset;

        // Converting Rotations to Rotation2d
        return Rotation2d.fromRotations(rotations);
    }

    /**
     * Gets the distance the swerve module has traveled.
     * 
     * @return The distance the swerve module has traveled as a double
     */
    public double getDistance() {
        // Getting rotations
        double rotations = drive.getPosition().getValue();

        // Converting Rotations to Meters
        return rotations * DriveConstants.driveRatio * DriveConstants.wheelCircumference;
    }

    /**
     * Gets the state of the swerve module.
     * 
     * @return the state of the swerve module.
     */
    public SwerveModuleState getState() {
        return this.state;
    }

    /**
     * Gets the position of the swerve module.
     * 
     * @return the position of the swerve module.
     */
    public SwerveModulePosition getPosition() {
        return this.position;
    }

    /**
     * Gets the velocity of the swerve module.
     * 
     * @return The velocity of the swerve module as a double.
     */
    public double getVelocity() {
        // Getting Rotations/Second
        double rps = drive.getVelocity().getValue();

        // Converting r/s to m/s
        return rps * DriveConstants.driveRatio * DriveConstants.wheelCircumference;
    }

    /**
     * Optimizes the angle to keep the motor from flipping.
     * 
     * @param targetAngle The angle to optimize.
     * @return The optimized angle as a Rotation2d.
     */
    public Rotation2d getAdjustedAngle(Rotation2d targetAngle) {
        Rotation2d theta = getAngle().minus(targetAngle);

        if (theta.getRadians() >= Math.PI) {
            theta.minus(new Rotation2d(2.0 * Math.PI));
        } if (theta.getRadians() <= -Math.PI) {
            theta.plus(new Rotation2d(2.0 * Math.PI));
        }

        return getAngle().minus(theta);
    }

    public void setState(SwerveModuleState state) {
        // Optimizing the SwerveModuleState
        state = SwerveModuleState.optimize(state, getAngle());

        // Updating speed and feedforward
        driveVolts.Velocity = state.speedMetersPerSecond / 2;
        driveVolts.FeedForward = feedForward.calculate(getVelocity(), getAcceleration());

        // Updating position and feedforward
        turnVolts.Position = getAdjustedAngle(state.angle).getRotations();
        turnVolts.FeedForward = feedForward.calculate(getVelocity(), getAcceleration());

        // Setting the speed and position of each motor
        drive.setControl(driveVolts);
        turn.setControl(turnVolts);
    }
}