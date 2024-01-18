package frc.robot;

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

        // Limits the maximum speed of the robot
        public static final double maxSpeed = 0.5;

        // The threshold of values where the robot will ignore joystick input. (including negatives)
        public static final double deadband = 0.2;

        // Current (amps) limit for drivetrain motors
        public static final int kCurrentLimit = 60;
    }

    public static class LauncherConstants {
        // CAN IDs for motor controllers
        public static final int FeederID = 5;
        public static final int LauncherID = 6;

        // Current (amps) limit for both Launcher wheels
        public static final int LauncherCurrentLimit = 80;
    }
}
