package frc.robot.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

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

    public SwerveModule(String name, int driveID, int turnID, int encID, double encOffset) {

        this.name = name;

        drive = new CANSparkMax(driveID, MotorType.kBrushless);
        turn = new CANSparkMax(turnID, MotorType.kBrushless);

        absEncoder = new DutyCycleEncoder(new DigitalInput(encID));
        this.encOffset = encOffset;

        drive.restoreFactoryDefaults();
        turn.restoreFactoryDefaults();

        drive.setInverted(false);
        turn.setInverted(false);

        drive.setIdleMode(IdleMode.kCoast);
        turn.setIdleMode(IdleMode.kCoast);

        driveRelEnc = drive.getEncoder();
        driveRelEnc.setPositionConversionFactor(DriveConstants.drivePosConversionFactor);
        driveRelEnc.setVelocityConversionFactor(DriveConstants.drivePosConversionFactor / 60.0);

        turnRelEnc = turn.getEncoder();
        turnRelEnc.setPositionConversionFactor(DriveConstants.turnPosConversionFactor);
        turnRelEnc.setVelocityConversionFactor(DriveConstants.turnPosConversionFactor / 60.0);
        
        drivePID = drive.getPIDController();
        drivePID.setP(DriveConstants.driveP, 0);
        drivePID.setI(DriveConstants.driveI, 0);
        drivePID.setD(DriveConstants.driveD, 0);
        drivePID.setFF(DriveConstants.driveFF, 0);

        turnPID = turn.getPIDController();
        turnPID.setP(DriveConstants.turnP, 0);
        turnPID.setI(DriveConstants.turnI, 0);
        turnPID.setD(DriveConstants.turnD, 0);
        turnPID.setFF(DriveConstants.turnFF, 0);

        drive.burnFlash();
        turn.burnFlash();
    }

    public void periodic() {
        SmartDashboard.putNumber(name + "/AbsPosition", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(name + "/TurnAngle", getHeading());
        SmartDashboard.putNumber(name + "/WheelSpeed", driveRelEnc.getVelocity());
        
    }

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

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(getHeading()));

        double desiredAngle = optimizedState.angle.getRadians();
        double adjustedAngle = getAdjustedAngle(desiredAngle);

        turnPID.setReference(adjustedAngle, ControlType.kPosition, 0);
        drivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity, 0);
    }

    public CANSparkMax getDrive() {
        return drive;
    }

    public CANSparkMax getTurn() {
        return turn;
    }

    public void stop() {
        drive.stopMotor();
        turn.stopMotor();
    }

}