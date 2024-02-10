package frc.robot.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import Util.CheesyUnits;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
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
    
    private final CANSparkMax drive, turn;
    private final RelativeEncoder driveRelEnc, turnRelEnc;
    private final SparkPIDController drivePID, turnPID;

    private final AnalogEncoder absEncoder;
    private final double encOffset;

    public SwerveModule(int driveID, int turnID, int encID, double encOffset) {
        drive = new CANSparkMax(driveID, MotorType.kBrushless);
        turn = new CANSparkMax(turnID, MotorType.kBrushless);

        absEncoder = new AnalogEncoder(encID);
        this.encOffset = encOffset;

        drive.restoreFactoryDefaults();
        turn.restoreFactoryDefaults();

        drive.setInverted(false);
        turn.setInverted(false);

        drive.setIdleMode(IdleMode.kCoast);
        turn.setIdleMode(IdleMode.kCoast);

        driveRelEnc = drive.getEncoder();
        driveRelEnc.setPositionConversionFactor(SwerveConstants.positionConversionFactor);
        driveRelEnc.setVelocityConversionFactor(SwerveConstants.positionConversionFactor/60.0);

        turnRelEnc = turn.getEncoder();
        turnRelEnc.setPositionConversionFactor(SwerveConstants.positionConversionFactor);
        turnRelEnc.setVelocityConversionFactor(SwerveConstants.positionConversionFactor/60.0);
        
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

        drive.burnFlash();
        turn.burnFlash();

    }

    public void periodic() {
        // TODO
        // Telemetry
    }

    public void initializeEncoder() {
        turnRelEnc.setPosition((absEncoder.getAbsolutePosition() - encOffset) - 1086.0/2.0);
    }

    /**
     * @return Returns the current heading of the module in cheesians
     */
    public double getHeading() {
        return turnRelEnc.getPosition()%1086.0;
    }

    /**
     * Used to enable continuous motion (prevents the module from spinning 360 degrees going from -180 to 180)
     * @param angle The desired angle of the module
     * @return The input angle to the turn PID
     */
    public double getAdjustedAngle(double angle) {
        double theta = getHeading() - angle;

        if (theta >= 1086.0/2.0) {
            theta-=1086.0;
        }
        if (theta <= -1086.0/2.0) {
            theta+=1086.0;
        }

        return turnRelEnc.getPosition() - theta;
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(CheesyUnits.cheesiansToRadians(getHeading())));

        double desiredAngle = CheesyUnits.radiansToCheesians(optimizedState.angle.getRadians());
        double adjustedAngle = getAdjustedAngle(desiredAngle);

        turnPID.setReference(adjustedAngle, ControlType.kPosition, 0);
        drivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity, 0);
    }

}
