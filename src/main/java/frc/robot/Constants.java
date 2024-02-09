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
        // IDs and offset
        public static final int frontLeftTurnID = 8;
        public static final int frontLeftDriveID = 7;
        public static final int frontLeftCancoderID = 1;
        public static final double frontLeftOffset = 0.0; // To be tuned

        public static final int frontRightTurnID = 6;
        public static final int frontRightDriveID = 5;
        public static final int frontRightCancoderID = 3;
        public static final double frontRightOffset = 0.0; // To be tuned

        public static final int backLeftTurnID = 2;
        public static final int backLeftDriveID = 1;
        public static final int backLeftCancoderID = 0;
        public static final double backLeftOffset = 0.0; // To be tuned

        public static final int backRightTurnID = 4;
        public static final int backRightDriveID = 3;
        public static final int backRightCancoderID = 2;
        public static final double backRightOffset = 0.0; // To be tuned

        // IDs for sensors
        public static final int gyroID = 4;

        // Speeds the robot can move at
        public static final double maxSpeed = 0.3; // Percent

        // The threshold of values where the robot will ignore joystick input. (including negatives)
        public static final double deadband = 0.5;
    
        // Gear ratios
        public static final double turnRatio = 150.0/7.0;
        public static final double driveRatio = 6.12;

        // Wheel Circumference
        public static final double wheelCircumference = Units.inchesToMeters(4 * Math.PI); // Meters

        // Kinematics
        public static final double kModuleToModuleDistance = Units.inchesToMeters(12); // Meters
        public static final double kModuleToCenter = kModuleToModuleDistance / 2; // Meters

        // PID Values
        public static final double driveP = 0.01; // To be tuned
        public static final double driveI = 0.0; // To be tuned
        public static final double driveD = 0.0; // To be tuned
        public static final double driveFF = 1.96; // To be tuned

        public static final double turnP = 0.01; // To be tuned
        public static final double turnI = 0.0; // To be tuned
        public static final double turnD = 0.01; // To be tuned
        public static final double turnFF = 0;
    }

    public static class LauncherConstants {
        // CAN IDs for motor controllers
        public static final int FeederID = 11;
        public static final int LauncherID = 12;

        // Current (amps) limit for both Launcher wheels
        public static final int LauncherCurrentLimit = 80;

        // Feed PID Constants
        public static final double feedP = 0.00001;
        public static final double feedI = 0;
        public static final double feedD = 0;
        public static final double feedFF = 0.01;

        // Launch PID Constants
        public static final double launchP = 0.00001;
        public static final double launchI = 0;
        public static final double launchD = 0;
        public static final double launchFF = 0.01;

        // The max speed for each motor in RPM
        public static final int feedSpeed = 15000; // Not quite the max speed, but it is necessary to launch the note
        public static final int launchSpeed = 5500;
    }
}
