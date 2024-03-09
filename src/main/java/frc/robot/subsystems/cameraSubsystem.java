package frc.robot.subsystems;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

public class cameraSubsystem extends SubsystemBase {
    static double pitch;
    static double area;
    static double yaw;
    PhotonCamera camera = new PhotonCamera("kitkam");
    PhotonPipelineResult result;
    PhotonTrackedTarget target;
    List<PhotonTrackedTarget> targets;
    Transform3d position;
    driveSubsystem driveSub;

    public cameraSubsystem() {
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
        if(result.hasTargets()){
            targets = result.getTargets();
            target = result.getBestTarget();

            pitch = target.getPitch();
            yaw = target.getYaw();
            area = target.getArea();
            position = target.getBestCameraToTarget();
            SmartDashboard.putNumber("Area", target.getArea());
        }
    }

    public static double getCameraYaw(){
        return yaw;
    }

    public static double getCameraPitch(){
        return pitch;
    }

    public static double getCameraArea(){
        return area;
    }

    public Transform3d cameraTransform(){
        return position;
    }

    //Suggested methods to have in DriveSubystem
    /*
    For Differential drive:
    public void driveAlignYaw(double yaw){
    MathUtil.applyDeadband(yaw, DriveConstants.YAW_DEADBAND);
    rightLeader.set(drivePIDYAW.calculate(yaw, 0) * DriveConstants.MAX_ALIGN_SPEED);
    leftLeader.set(-drivePIDYAW.calculate(yaw, 0) * DriveConstants.MAX_ALIGN_SPEED);
    } 

    For Swerve Drive Command:
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            x_trans.getAsDouble() * (DriveConstants.maxWheelVelocity + (marioKart.getAsBoolean() ? Math.signum(x_trans.getAsDouble()) * ((DriveConstants.maxWheelVelocity-speeds.vxMetersPerSecond)/DriveConstants.maxWheelVelocity * DriveConstants.maxWheelVelocity) : 0.0)), 
            y_trans.getAsDouble() * (DriveConstants.maxWheelVelocity + (marioKart.getAsBoolean() ? Math.signum(y_trans.getAsDouble()) * ((DriveConstants.maxWheelVelocity-speeds.vyMetersPerSecond)/DriveConstants.maxWheelVelocity * DriveConstants.maxWheelVelocity) : 0.0)),
            PID.calculate(MathUtil.applyDeadband(yaw, 0.5)) * DriveConstants.maxRotationalVelocity, 
            drivetrain.getYaw()));
    */
}
