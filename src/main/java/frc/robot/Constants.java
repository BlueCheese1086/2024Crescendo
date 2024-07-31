package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class BeamConstants {
        // Sensor IDs
        public static final int shooterID = 0;
        public static final int feedID = 1;
    }

    public static class DriveConstants {
        // Motor IDs
        public static final int flDriveID = 1;
        public static final int flTurnID  = 2;
        public static final int frDriveID = 3;
        public static final int frTurnID  = 4;
        public static final int blDriveID = 5;
        public static final int blTurnID  = 6;
        public static final int brDriveID = 7;
        public static final int brTurnID  = 8;

        // Sensor IDs
        public static final int flCancoderID =  2;
        public static final int frCancoderID =  3;
        public static final int blCancoderID =  1;
        public static final int brCancoderID =  0;
        public static final int gyroID       = 10;

        // Encoder offsets
        public static final double flOffset = 0.249; // Rotations
        public static final double frOffset = 0.420; // Rotations
        public static final double blOffset = 0.674; // Rotations
        public static final double brOffset = 0.513; // Rotations

        // Kinematics
        public static final Translation2d flTranslation = new Translation2d(-DriveConstants.width / 2,  DriveConstants.length / 2);
        public static final Translation2d frTranslation = new Translation2d( DriveConstants.width / 2,  DriveConstants.length / 2);
        public static final Translation2d blTranslation = new Translation2d(-DriveConstants.width / 2, -DriveConstants.length / 2);
        public static final Translation2d brTranslation = new Translation2d( DriveConstants.width / 2, -DriveConstants.length / 2);

        // Max Speeds
        public static final double maxDriveSpeed = Units.feetToMeters(20.4); // Meters / Second
        public static final double maxTurnSpeed  = Math.PI; // Radians / Second

        // Robot Measurements
        public static final double width = Units.inchesToMeters(33); // Meters
        public static final double length = Units.inchesToMeters(33); // Meters
        public static final double wheelRadius = Units.inchesToMeters(2); // Meters
        public static final double wheelCircumference = 2 * Math.PI * wheelRadius; // Meters
        public static final double driveRatio = 6.75; // Gear Ratio
        public static final double turnRatio = 12.8; // Gear Ratio

        // Drive PID Values
        public static final double driveP = 0.01;
        public static final double driveI = 0;
        public static final double driveD = 0;
        public static final double driveFF = 0.2;

        // Turn PID Values
        public static final double turnP = 0.5;
        public static final double turnI = 0;
        public static final double turnD = 0;
        public static final double turnFF = 0;

        // Conversion factors
        public static final double drivePosConversionFactor = 2.0 * Math.PI * wheelRadius / driveRatio;
        public static final double driveVelConversionFactor = 2.0 * Math.PI * wheelRadius / driveRatio / 60.0;
        public static final double turnPosConversionFactor  = 2.0 * Math.PI / turnRatio;
        public static final double turnVelConversionFactor  = 2.0 * Math.PI / turnRatio / 60.0;
    }

    public class IntakeConstants {
        // Motor IDs
        public static final int accessID = 11;
        public static final int rollerID = 12;

        // Access PID Values
        public static final double accessP = 0.5;
        public static final double accessI = 0;
        public static final double accessD = 0;
        public static final double accessFF = 0;

        // Intake Specs
        public static final double accessGearRatio = 24.0 / 11.0;

        // Conversion factors
        public static final double anglePosConversionFactor = 2.0 * Math.PI / 54.55;
        public static final double absPosConversionFactor = 2.0 * Math.PI / ((48.0 / 44.0) * (48.0 / 24.0));
    }

    public class ShooterConstants {
        // Motor IDs
        public static final int feedID = 21;
        public static final int pivotID = 22;
        public static final int lLauncherID = 23;
        public static final int rLauncherID = 24;

        // Robot Measurements
        public static final double alignGearRatio = 4.5;

        // PID Values
        public static final double kP = 0.01;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0.001;

        // Max speeds
        public static final double maxShootSpeed = 1;   // Duty Cycle
        public static final double maxFeedSpeed = 1;    // Duty Cycle
        public static final double maxAlignSpeed = 0.5; // Duty Cycle

        // Conversion factors
        public static final double alignPosConversionFactor = 2 * Math.PI;
    }

    public class TowerConstants {
        // Motor IDs
        public static final int lTowerID = 31;
    }
}