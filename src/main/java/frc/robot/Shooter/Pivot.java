package frc.robot.Shooter;

import java.util.List;
import java.util.Objects;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Vision.Vision;

public class Pivot extends SubsystemBase {
    // Motor controllers
    private CANSparkMax align = new CANSparkMax(ShooterConstants.alignID, MotorType.kBrushless);

    // Encoders
    private RelativeEncoder alignEncoder;

    // PID Controllers
    private SparkPIDController alignPID;

    // This subsystem relies on the vision subsystem to aim.
    private Vision vision;


    private static Pivot instance;

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

    //states
    public static Pivot getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (Objects.isNull(instance)) instance = new Pivot();

        return instance;
    }

    public Pivot() {
        // Getting an instance of the Vision subsystem.
        vision = Vision.getInstance();

        // Resetting the settings of the sparkmaxes
        align.restoreFactoryDefaults();

        // Setting the idle mode of the sparkmaxes
        align.setIdleMode(IdleMode.kCoast);

        // Saving the settings of the sparkmaxes
        align.burnFlash();

        // Configuring the align encoder
        alignEncoder = align.getAlternateEncoder(Type.kQuadrature, 8192);

        // Setting the conversion factors for the align encoder
        alignEncoder.setPositionConversionFactor(ShooterConstants.alignPosConversionFactor);

        // Configuring the align PID
        alignPID = align.getPIDController();

        // Setting PIDFF values
        alignPID.setP(ShooterConstants.kP);
        alignPID.setI(ShooterConstants.kI);
        alignPID.setD(ShooterConstants.kD);
        alignPID.setFF(ShooterConstants.kFF);
    }

    /** Resets the encoder of the align motor. */
    public void resetEncoders() {
        alignEncoder.setPosition(0);
    }

    public Rotation2d getAngle() {
        return new Rotation2d(alignEncoder.getPosition());
    }

    public Rotation2d calculateAngle() {
        // Getting the targets from the vision system
        List<PhotonTrackedTarget> targets = vision.getTargets();

        // Checking the alliance so we don't align to the wrong apriltag.
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            for (PhotonTrackedTarget target : targets) {
                // Checking the ID of the apriltag.
                if (target.getFiducialId() != 7) {
                    break;
                }

                // Getting the necessary angle while accounting for the camera's offset.
                Rotation2d angle = Rotation2d.fromDegrees(target.getPitch());
                return angle.plus(VisionConstants.camRotation.toRotation2d());
            }
        } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            for (PhotonTrackedTarget target : targets) {
                // Checking the ID of the apriltag.
                if (target.getFiducialId() != 4) {
                    break;
                }

                // Getting the necessary angle while accounting for the camera's offset.
                Rotation2d angle = Rotation2d.fromDegrees(target.getPitch());
                return angle.plus(VisionConstants.camRotation.toRotation2d());
            }
        }

        // No allaince?  Weirdo...
        return new Rotation2d();
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
