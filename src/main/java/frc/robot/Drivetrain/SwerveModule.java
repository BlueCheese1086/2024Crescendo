package frc.robot.Drivetrain;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {

    /*           backLeft    frontLeft
     *                 x_____x
     *                 |     |
     * --------shooter-----------intake--- x-axis
     *                 |_____|
     *                 x     x
     *           backRight   frontRight
     */
    
    private final String name;

    private final CANSparkMax drive, turn;
    private final RelativeEncoder driveRelEnc, turnRelEnc;
    private final SparkPIDController drivePID, turnPID;

    private final DutyCycleEncoder absEncoder;
    private final double encOffset;

    private SwerveModuleState state = new SwerveModuleState();

    public SwerveModule(String name, int driveID, int turnID, int encID, double encOffset) {

        this.name = name;

        drive = new CANSparkMax(driveID, MotorType.kBrushless);
        turn = new CANSparkMax(turnID, MotorType.kBrushless);

        absEncoder = new DutyCycleEncoder(new DigitalInput(encID));
        this.encOffset = encOffset;

        drive.restoreFactoryDefaults();
        turn.restoreFactoryDefaults();

        drive.setSmartCurrentLimit(60);
        turn.setSmartCurrentLimit(30);

        drive.setInverted(false);
        turn.setInverted(false);

        drive.setIdleMode(IdleMode.kCoast);
        turn.setIdleMode(IdleMode.kBrake);

        driveRelEnc = drive.getEncoder();
        driveRelEnc.setPosition(0.0);
        driveRelEnc.setPositionConversionFactor(SwerveConstants.drivePosConversionFactor);
        driveRelEnc.setVelocityConversionFactor(SwerveConstants.drivePosConversionFactor / 60.0);

        turnRelEnc = turn.getEncoder();
        turnRelEnc.setPositionConversionFactor(SwerveConstants.steerPosConversionFactor);
        turnRelEnc.setVelocityConversionFactor(SwerveConstants.steerPosConversionFactor / 60.0);
        
        drivePID = drive.getPIDController();
        drivePID.setP(SwerveConstants.kPDriveVelo, 0);
        drivePID.setI(SwerveConstants.kIDriveVelo, 0);
        drivePID.setD(SwerveConstants.kDDriveVelo, 0);
        drivePID.setFF(SwerveConstants.kFFDriveVelo, 0);

        turnPID = turn.getPIDController();
        turnPID.setP(SwerveConstants.kPTurn, 0);
        turnPID.setI(SwerveConstants.kITurn, 0);
        turnPID.setD(SwerveConstants.kDTurn, 0);
        turnPID.setFF(SwerveConstants.kFFTurn, 0);

        // new DebugPID(drivePID, "Drive/Drive");
        // new DebugPID(turnPID, "Drive/Turn");

        drive.burnFlash();
        turn.burnFlash();
    }

    /**
     * Publishes telemetry
     */
    public void periodic() {
        // TODO
        // Telemetry
        Logger.recordOutput(name + "/Drive/MotorCurrent", drive.getOutputCurrent());
        Logger.recordOutput(name + "/Drive/BusVoltage", drive.getBusVoltage());
        Logger.recordOutput(name + "/Drive/Velocity", driveRelEnc.getVelocity());
        Logger.recordOutput(name + "/Drive/Temperature", drive.getMotorTemperature());

        Logger.recordOutput(name + "/Turn/MotorCurrent", turn.getOutputCurrent());
        Logger.recordOutput(name + "/Turn/BusVoltage", turn.getBusVoltage());
        Logger.recordOutput(name + "/Turn/Position", turnRelEnc.getPosition());
        Logger.recordOutput(name + "/Turn/Temperature", turn.getMotorTemperature());

        SmartDashboard.putNumber(name + "/AbsPosition", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(name + "/TurnAngle", getHeading());
        SmartDashboard.putNumber(name + "/WheelSpeed", driveRelEnc.getVelocity());
        SmartDashboard.putNumber(name + "/StateMS", state.speedMetersPerSecond);
        
    }

    /**
     * Sets the relative encoder to the value measured by the absolute encoder
     */
    public void initializeEncoder() {
        turnRelEnc.setPosition((absEncoder.getAbsolutePosition() - encOffset) * (2.0 * Math.PI));
    }

    /**
     * @return Returns the current heading of the module in cheesians
     */
    public double getHeading() {
        return turnRelEnc.getPosition()%(2.0 * Math.PI);
    }

    /**
     * @return Returns the position of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveRelEnc.getPosition(), new Rotation2d(getHeading()));
    }

    /**
     * Used to enable continuous motion (prevents the module from spinning 360 degrees going from -180 to 180)
     * @param angle The desired angle of the module
     * @return The input angle to the turn PID
     */
    public double getAdjustedAngle(double angle) {
        double theta = getHeading() - angle;

        if (theta >= Math.PI) {
            theta-=(2.0 * Math.PI);
        }
        if (theta <= -Math.PI) {
            theta+=(2.0 * Math.PI);
        }

        return turnRelEnc.getPosition() - theta;
    }

    /**
     * Sets the desired state of the module with a feedback controller
     * @param state The desired state of the module
     */
    public void setState(SwerveModuleState state) {
        this.state = state;
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(getHeading()));

        double desiredAngle = optimizedState.angle.getRadians();
        double adjustedAngle = getAdjustedAngle(desiredAngle);

        turnPID.setReference(adjustedAngle, ControlType.kPosition, 0);
        drivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity, 0);
    }

    /**
     * @return Returns the current state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveRelEnc.getVelocity(), new Rotation2d(getHeading()));
    }

    /**
     * @return Returns the current being drawn by the turn motor
     */
    public double getTurnCurrent() {
        return turn.getOutputCurrent();
    }

    /**
     * @return Returns the current being drawn by the drive motor
     */
    public double getDriveCurrent() {
        return drive.getOutputCurrent();
    }

    /**
     * @return Returns the drive motor instance
     */
    public CANSparkMax getDrive() {
        return drive;
    }

    /**
     * @return Returns the turn motor instance
     */
    public CANSparkMax getTurn() {
        return turn;
    }

    /**
     * Stops the turn and drive motors
     */
    public void stop() {
        drive.stopMotor();
        turn.stopMotor();
    }

}
