package frc.robot.Sensors;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Gyro extends SubsystemBase {

    private static Gyro instance;

    private Pigeon2 pigeon;

    private double rollOffset, pitchOffset;
    private double prevYaw, yawVelo;

    private final Timer timer = new Timer();

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro(DriveConstants.pigeonID);
        }
        return instance;
    }

    public Gyro(int id) {
        pigeon = new Pigeon2(id);
        timer.start();
    }

    public void initGyro() {
        pigeon.configFactoryDefault();
        // The gyro is offset by 180deg
        try {
            pigeon.setYaw(DriverStation.getAlliance().get() == Alliance.Red ? 0 : 180);
        } catch (Exception e) {
            pigeon.setYaw(0.0);
        }
        
        yawVelo = 0.0;
        prevYaw = pigeon.getYaw();
        rollOffset = pigeon.getRoll();
        pitchOffset = pigeon.getPitch();
    }

    @Override
    public void periodic() {
        timer.stop();
        yawVelo = (pigeon.getYaw() - prevYaw)/timer.get();
        SmartDashboard.putNumber("Gyro/PrevYaw", prevYaw);
        prevYaw = pigeon.getYaw();

        SmartDashboard.putNumber("Gyro/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Gyro/Pitch", getPitch());
        SmartDashboard.putNumber("Gyro/AngularVelocity", getYawVelocity().getDegrees());
        SmartDashboard.putNumber("Gyro/Period", timer.get());
        SmartDashboard.putNumber("Gyro/Roll", getRoll());
        SmartDashboard.putNumber("Gyro/Yaw", pigeon.getYaw());
        SmartDashboard.putNumber("Gyro/RotationAtHeading0", getPitchAtHeading(new Rotation2d()).getDegrees());
        timer.reset();
        timer.start();
    }

    /**
     * Gets the pigeon angle
     * [-180 deg, 180 deg] 0 IS POINTING AWAY FROM BLUE ALLIANCE
     * @return Rotation2d of the gyro angle
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    public Rotation2d getYawVelocity() {
        return Rotation2d.fromDegrees(yawVelo);
    }

    public double getPitch() {
        return pigeon.getPitch() - pitchOffset;
    }

    public double getRoll() {
        return pigeon.getRoll() - rollOffset;
    }

    public Rotation2d getPitchAtHeading(Rotation2d heading) {

        // These two are swapped I hate it too
        double pitch = pigeon.getRoll();
        double roll = pigeon.getPitch();
        Rotation2d angleDelta = getAngle().minus(heading);
        // System.out.println(angleDelta.getDegrees());

        double pitchAtHeading = pitch*angleDelta.getCos()*angleDelta.getCos()*Math.signum(-angleDelta.getCos()) + roll*angleDelta.getSin()*angleDelta.getSin()*Math.signum(angleDelta.getSin());

        return Rotation2d.fromDegrees(pitchAtHeading);
    }

    public void setAngle(double a) {
        pigeon.setYaw(a);
    }

    public Pigeon2 getGyro() {
        return pigeon;
    }
    
}