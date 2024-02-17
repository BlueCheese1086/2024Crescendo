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
    public static class OperatorConstants {
        // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB tab of the DriverStation
        public static final int PrimaryPort = 0;
        public static final int SecondaryPort = 1;
    }

    public static class DriveConstants {
        // CAN IDs for motor controllers
        public static final int FrontLeftID = 2;
        public static final int BackLeftID = 1;
        public static final int FrontRightID = 4;
        public static final int BackRightID = 3;

        // IDs for sensors
        public static final int gyroID = 4;

        // Speeds the robot can move at
        public static final double maxSpeed = 0.5; // Duty Cycle
        public static final double maxDriveSpeed = 3; // RPM
        public static final double maxTurnSpeed = 3; // RPM

        // The threshold of values where the robot will ignore joystick input. (including negatives)
        public static final double deadband = 0.2;

        // Current (amps) limit for drivetrain motors
        public static final int currentLimit = 35;
        public static final double voltageCompensation = 12;

        // Wheel Circumference
        public static final double wheelCircumference = Units.inchesToMeters(4 * Math.PI); // Meters

        // Kinematics
        public static final double kModuleToModuleDistance = Units.inchesToMeters(19.750); // Meters
        public static final double kModuleToCenter = kModuleToModuleDistance / 2; // Meters
    }

    public static class LauncherConstants {
        // CAN IDs for motor controllers
        public static final int FeederID = 11;
        public static final int LauncherID = 12;

        // Current (amps) limit for both Launcher wheels
        public static final int LauncherCurrentLimit = 80;

        // Feed PID Constants
        public static final double feedP = 0.001;
        public static final double feedI = 0;
        public static final double feedD = 0;
        public static final double feedFF = 0.01;

        // Launch PID Constants
        public static final double launchP = 0.001;
        public static final double launchI = 0;
        public static final double launchD = 0;
        public static final double launchFF = 0.01;

        // The max speed for each motor in RPM
        public static final int feedSpeed = 15000; // Not quite the max speed, but it is necessary to launch the note
        public static final int launchSpeed = 5500;
    }
}
