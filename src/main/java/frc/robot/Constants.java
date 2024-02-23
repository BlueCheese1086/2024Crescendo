package frc.robot;

import edu.wpi.first.math.util.Units;

/** Contains most of the configurations that the robot depends on. */
public final class Constants {
    public static class OperatorConstants {
        // Port numbers for driver and operator joysticks.
        public static final int PrimaryPort = 0;
        public static final int SecondaryPort = 1;
    }

    public static class DriveConstants {
        // CAN IDs for motor controllers
        public static final int FrontLeftID = 1;
        public static final int BackLeftID = 2;
        public static final int FrontRightID = 3;
        public static final int BackRightID = 4;

        // IDs for sensors
        public static final int gyroID = 2;

        // Speeds the robot can move at
        public static final double maxSpeed = 0.5;
        public static final double maxDriveSpeed = 3;
        public static final double maxTurnSpeed = 3;

        // The threshold of values where the robot will ignore joystick input.
        // (including negatives)
        public static final double deadband = 0.2;

        // Current (amps) limit for drivetrain motors
        public static final int currentLimit = 35;
        public static final double voltageCompensation = 12;

        // Wheel Circumference
        public static final double wheelCircumference = Units.inchesToMeters(4 * Math.PI);

        // Gear Ratio
        public static final double gearRatio = 10.71;

        // Kinematics
        public static final double kModuleToModuleDistance = Units.inchesToMeters(19.750);
        public static final double kModuleToCenter = kModuleToModuleDistance / 2;
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

        // Wheel Specs
        public static final double wheelRadius = Units.inchesToMeters(2);
        public static final double wheelCircumference = Math.pow(wheelRadius, 2) * Math.PI;

        // Conversion factors for R/M to M/S
        public static final double feedConversionFactor = wheelCircumference; // Nothing to divide by because gear ratios are 1:1
        public static final double launchConversionFactor = wheelCircumference;
    }
}
