package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class driveSubsystem extends SubsystemBase {
  CANSparkMax rightLeader = new CANSparkMax(Constants.DriveConstants.FRONT_RIGHT_ID, MotorType.kBrushless); 
  CANSparkMax rightFollower = new CANSparkMax(Constants.DriveConstants.BACK_RIGHT_ID, MotorType.kBrushless);
  CANSparkMax leftLeader = new CANSparkMax(Constants.DriveConstants.FRONT_LEFT_ID, MotorType.kBrushless);
  CANSparkMax leftFollower = new CANSparkMax(Constants.DriveConstants.BACK_LEFT_ID, MotorType.kBrushless);

  RelativeEncoder encoderFR = rightLeader.getEncoder();
  RelativeEncoder encoderFL = leftLeader.getEncoder();
  RelativeEncoder encoderBR = rightFollower.getEncoder();
  RelativeEncoder encoderBL = leftFollower.getEncoder();

  SparkPIDController FLPID = leftLeader.getPIDController();
  SparkPIDController FRPID = rightLeader.getPIDController();

  PIDController drivePIDYAW = new PIDController(0.1, 0, 0);
  PIDController drivePIDPITCH = new PIDController(0.3, 0, 0);

  ChassisSpeeds superChassisSpeeds = new ChassisSpeeds(0, 0, 0);

  CommandXboxController xbox = new CommandXboxController(0);
  XboxController joy = xbox.getHID();

  PigeonIMU gyro = new PigeonIMU(2);
  Field2d field = new Field2d();
  Pose2d pose = new Pose2d();
  boolean isRed = DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

  DifferentialDriveOdometry odometry;
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
  DifferentialDriveWheelSpeeds difSpeeds = new DifferentialDriveWheelSpeeds();

  public driveSubsystem() {

    rightLeader.restoreFactoryDefaults();
    leftLeader.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();

    rightLeader.setIdleMode(IdleMode.kBrake);
    leftLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);

    leftLeader.setSmartCurrentLimit(DriveConstants.DRIVETRAINLIMITS);
    leftFollower.setSmartCurrentLimit(DriveConstants.DRIVETRAINLIMITS);
    rightLeader.setSmartCurrentLimit(DriveConstants.DRIVETRAINLIMITS);
    rightFollower.setSmartCurrentLimit(DriveConstants.DRIVETRAINLIMITS);

    leftLeader.setInverted(false);
    rightLeader.setInverted(true);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader); 

    FLPID.setP(DriveConstants.DriveP);
    FLPID.setI(DriveConstants.DriveI);
    FLPID.setD(DriveConstants.DriveD);
    FLPID.setFF(DriveConstants.DriveFF);

    FRPID.setP(DriveConstants.DriveP);
    FRPID.setI(DriveConstants.DriveI);
    FRPID.setD(DriveConstants.DriveD);
    FRPID.setFF(DriveConstants.DriveFF);

    encoderFL.setPosition(0);
    encoderFR.setPosition(0);
    gyro.setYaw(0);
    if(isRed){
      pose = new Pose2d((DriveConstants.fieldMaxX - DriveConstants.startingX), DriveConstants.startingY, Rotation2d.fromDegrees(gyro.getYaw()));
    }
    else{
      pose = new Pose2d(DriveConstants.startingX, DriveConstants.startingY, Rotation2d.fromDegrees(gyro.getYaw()));
    }
    odometry = new DifferentialDriveOdometry(new Rotation2d(), encoderFL.getPosition(), encoderFR.getPosition(), pose);
    encoderFL.setPositionConversionFactor(DriveConstants.driveRatio);
    encoderFR.setPositionConversionFactor(DriveConstants.driveRatio);
    encoderFL.setVelocityConversionFactor(DriveConstants.driveRatio / 60.0);
    encoderFR.setVelocityConversionFactor(DriveConstants.driveRatio / 60.0);

    AutoBuilder.configureRamsete(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getSpeeds, // Current ChassisSpeeds supplier
      (speeds) -> {
        SmartDashboard.putNumber("Pathplanner vx", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Pathplanner vy", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Pathplanner Omega Radians", speeds.omegaRadiansPerSecond);
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        driveChassis(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
      },
      0.2,
      0.5,
      new ReplanningConfig(),
      () -> DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red),
      this
    );
  }

  @Override
  public void periodic() {
    pose = odometry.update(Rotation2d.fromDegrees(gyro.getYaw()), getPositions());
    field.setRobotPose(pose);
    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("LSpeed", encoderFL.getVelocity());
    SmartDashboard.putNumber("RSpeed", encoderFR.getVelocity());
    SmartDashboard.putNumber("Pose X", pose.getX());
    SmartDashboard.putNumber("Pose Y", pose.getY());
  }

  /*public void set(double rotateSpeed, double driveSpeed) {
    rightLeader.set((driveSpeed + rotateSpeed));
    leftLeader.set((driveSpeed - rotateSpeed));
    
    SmartDashboard.putNumber("Rotate speed:", rotateSpeed);
    SmartDashboard.putNumber("Drive speed:", driveSpeed);
    SmartDashboard.putNumber("Joystick x", joy.getLeftX());
    SmartDashboard.putNumber("Joystick y", joy.getLeftY());
    SmartDashboard.putNumber("FL Rotations", encoderFL.getPosition());
    SmartDashboard.putNumber("FR Rotations", encoderFR.getPosition());
    SmartDashboard.putNumber("Counts per rev", encoderFL.getCountsPerRevolution());
  }*/

  public RelativeEncoder getEncoderFL(){
    return encoderFL;
  }

  public RelativeEncoder getEncoderFR(){
    return encoderFR;
  }

  public void driveAlignYaw(double yaw){
    MathUtil.applyDeadband(yaw, DriveConstants.YAW_DEADBAND);
    rightLeader.set(drivePIDYAW.calculate(yaw, 0) * DriveConstants.MAX_ALIGN_SPEED);
    leftLeader.set(-drivePIDYAW.calculate(yaw, 0) * DriveConstants.MAX_ALIGN_SPEED);
  }

  public void driveAlignPitch(double pitch){
    rightLeader.set(drivePIDPITCH.calculate(pitch, 3) * DriveConstants.MAX_ALIGN_SPEED);
    leftLeader.set(drivePIDPITCH.calculate(pitch, 3) * DriveConstants.MAX_ALIGN_SPEED);
  }

  public Pose2d getPose(){
    return pose;
  }

  public void resetPose(Pose2d Newpose){
    odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), encoderFL.getPosition(), encoderFR.getPosition(), Newpose);
  }

  public void driveChassis(double leftSpeed, double rightSpeed){
    FLPID.setReference(leftSpeed, ControlType.kVelocity);
    FRPID.setReference(rightSpeed, ControlType.kVelocity);
    double[] xyz_accel = new double[3];
    gyro.getAccelerometerAngles(xyz_accel);
    SmartDashboard.putNumber("Dif Left", leftSpeed);
    SmartDashboard.putNumber("Dif Right", rightSpeed);
    SmartDashboard.putNumber("FL Meters", encoderFL.getPosition());
    SmartDashboard.putNumber("FR Meters", encoderFR.getPosition());
    SmartDashboard.putNumber("Rotation 2d", gyro.getYaw());
    SmartDashboard.putNumber("X distance", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y distance", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Rotation", odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("Acceleration x", xyz_accel[0]);
    SmartDashboard.putNumber("Acceleration y", xyz_accel[1]);
    SmartDashboard.putNumber("Acceleration z", xyz_accel[2]);
  }


  /*public void drive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds diffSpeeds = kinematics.toWheelSpeeds(speeds);
    SmartDashboard.putNumber("Left Speed", diffSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Speed", diffSpeeds.rightMetersPerSecond);

    FLPID.setReference(diffSpeeds.leftMetersPerSecond, ControlType.kVelocity);
    FRPID.setReference(diffSpeeds.rightMetersPerSecond, ControlType.kVelocity);
  }*/

  public ChassisSpeeds getSpeeds(){
    return kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(encoderFL.getVelocity(),encoderFR.getVelocity()));
  }

  public DifferentialDriveWheelPositions getPositions() {
    return new DifferentialDriveWheelPositions(encoderFL.getPosition(), encoderFR.getPosition());
  }

  public void resetOdometry(){
    encoderFL.setPosition(0);
    encoderFR.setPosition(0);
    gyro.setYaw(0);
    pose = new Pose2d(2, 7, Rotation2d.fromDegrees(gyro.getYaw()));

    odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), getPositions(), pose);
  }
}
