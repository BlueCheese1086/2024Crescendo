package frc.robot.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
    private final AnalogEncoder cancoder;
    private final double offset;

    private final CANSparkMax turn;
    private final CANSparkMax drive;

    private final RelativeEncoder turnEnc;
    private final RelativeEncoder driveEnc;

    private final SparkPIDController turnPID;
    private final SparkPIDController drivePID;

    private SwerveModuleState state;
    private String name;

    public SwerveModule(String name, int driveId, int turnId, int channel, double offset) {
        this.name = name;
        this.state = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

        cancoder = new AnalogEncoder(channel);
        this.offset = offset;

        turn = new CANSparkMax(turnId, MotorType.kBrushless);
        drive = new CANSparkMax(driveId, MotorType.kBrushless);

        turn.restoreFactoryDefaults();
        drive.restoreFactoryDefaults();

        turn.setSmartCurrentLimit(35);
        drive.setSmartCurrentLimit(35);

        drive.enableVoltageCompensation(12);
        turn.enableVoltageCompensation(12);

        turn.setInverted(true);
        drive.setInverted(false);

        turn.setIdleMode(IdleMode.kCoast);
        drive.setIdleMode(IdleMode.kCoast);

        turnEnc = turn.getEncoder();
        driveEnc = drive.getEncoder();

        driveEnc.setPosition(0);
        turnEnc.setPosition(0);

        turnPID = turn.getPIDController();
        drivePID = drive.getPIDController();

        turnPID.setP(DriveConstants.turnP);
        turnPID.setI(DriveConstants.turnI);
        turnPID.setD(DriveConstants.turnD);

        drivePID.setP(DriveConstants.driveP);
        drivePID.setI(DriveConstants.driveI);
        drivePID.setD(DriveConstants.driveD);
        drivePID.setFF(DriveConstants.driveFF);

        turnEnc.setPositionConversionFactor(360.0 / DriveConstants.turnRatio);
        driveEnc.setVelocityConversionFactor(DriveConstants.wheelCircumference / DriveConstants.driveRatio / 60);
        driveEnc.setPositionConversionFactor(DriveConstants.wheelCircumference / DriveConstants.driveRatio);

        turn.burnFlash();
        drive.burnFlash();
    }

    /** Runs every 20 ms */
    @Override
    public void periodic() {}

    /** Initializes the turn encoders to match the cancoder. */
    public void initEncoder() {
        turnEnc.setPosition((cancoder.getAbsolutePosition() - offset) * 360.0);
    }

    /**
     * Calculate the angle motor setpoint based on the desired angle and the current
     * angle measurement
     * 
     * @return Returns the calculated delta angle
     */
    public double getAdjustedAngle(double targetAngle) {
        double theta = getTurnAngle().getDegrees() - targetAngle;

        if (theta > 180)  theta -= 360;
        if (theta < -180) theta += 360;

        return turnEnc.getPosition() - theta;
    }

    /**
     * Gets the current module angle based on relative encoder
     * 
     * @return Rotation2d of the current module angle
     */
    public Rotation2d getTurnAngle() {
        return Rotation2d.fromDegrees((turnEnc.getPosition() % 360 + 360) % 360);
    }

    /**
     * Sets the module to the desired state
     * 
     * @param state The desired state
     */
    public void setState(SwerveModuleState state) {
        this.state = SwerveModuleState.optimize(state, getTurnAngle());

        drivePID.setReference(this.state.speedMetersPerSecond, ControlType.kVelocity);
        turnPID.setReference(getAdjustedAngle(this.state.angle.getDegrees()), ControlType.kPosition);
    }
}