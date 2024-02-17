package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DriveConstants {
        // Motor IDs and offsets
        public static final int frontLeftTurnID = 1;
        public static final int frontLeftDriveID = 2;
        public static final int frontLeftCancoderID = 9;
        public static final double frontLeftOffset = 0.007;

        public static final int backLeftTurnID = 3;
        public static final int backLeftDriveID = 4;
        public static final int backLeftCancoderID = 1;
        public static final double backLeftOffset = 0.079;

        public static final int frontRightTurnID = 7;
        public static final int frontRightDriveID = 8;
        public static final int frontRightCancoderID = 8;
        public static final double frontRightOffset = 0.498;

        public static final int backRightTurnID = 5;
        public static final int backRightDriveID = 6;
        public static final int backRightCancoderID = 0;
        public static final double backRightOffset = 0.594;

        public static final int gyroID = 2;

        // Speeds the robot can move at
        public static final double maxDriveSpeed = 0.25; // Meters/second
        public static final double maxTurnSpeed = 5; // Radians/second

        // The threshold of values where the robot will ignore joystick input. (including negatives)
        public static final double deadband = 0.2;
    
        // Gear ratios
        public static final double turnRatio = 12.8;
        public static final double driveRatio = 5.14;

        // Wheel Circumference
        public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI; // Meters

        // Encoder Conversion Factors
        public static final double drivePosConversionFactor = DriveConstants.wheelCircumference / DriveConstants.driveRatio;
        public static final double turnPosConversionFactor = 360 / DriveConstants.turnRatio;

        // Kinematics
        public static final double kModuleToModuleDistance = Units.inchesToMeters(19.5); // Meters
        public static final double kModuleToCenter = kModuleToModuleDistance / 2.0; // Meters

        // Drive PID Constants
        public static final double driveP = 0.01;
        public static final double driveI = 0.0;
        public static final double driveD = 0.0;
        public static final double driveFF = 1.96;

        // Turn PID Constants
        public static final double turnP = 1.0;
        public static final double turnI = 0.0;
        public static final double turnD = 0.0;
        public static final double turnFF = 0.0;
    }

    public static class LauncherConstants {
        // CAN IDs for motor controllers
        public static final int FeedID = 12;
        public static final int LaunchID = 11;

        // Current (amps) limit for both Launcher wheels
        public static final int CurrentLimit = 80;

        // Feed PID Constants
        public static final double feedP = 0.0001;
        public static final double feedI = 0;
        public static final double feedD = 0;
        public static final double feedFF = 0.01;

        // Launch PID Constants
        public static final double launchP = 0.0001;
        public static final double launchI = 0;
        public static final double launchD = 0;
        public static final double launchFF = 0.01;

        // The max speed for each motor in RPM
        public static final int maxFeedInSpeed = 5800;//1;
        public static final int maxFeedOutSpeed = 15000;//3;
        public static final int maxLaunchSpeed = 5800;//1;
    }

    public static class ClimbConstants {
        // Motor IDs
        public static final int leftID = 21;
        public static final int rightID = 22;

        // Gear Ratios
        public static final double leftRatio = 25;
        public static final double rightRatio = 25;

        // The max speed the climb will run at in percent
        public static final double maxSpeed = 0.3;
    }

    public static class IntakeConstants {
        // Motor IDs
        public static final int accessID = 31;
        public static final int rollerID = 32;

        // Gear Ratios
        public static final double accessRatio = 26.18;
        public static final double rollerRatio = 1;

        // The max speed the intake will run at in percent
        public static final double maxSpeed = 0.5;
    }
}
