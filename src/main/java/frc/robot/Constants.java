package frc.robot;

import edu.wpi.first.math.util.Units;

/** Contains most of the configurations that the robot depends on. */
public final class Constants {
    public static class DriveConstants {
        // CAN IDs for motor controllers
        public static final int frontLeftID = 1;
        public static final int backLeftID = 2;
        public static final int frontRightID = 3;
        public static final int backRightID = 4;

        // IDs for sensors
        public static final int gyroID = 2;

        // Speeds the robot can move at (MPS)
        public static final double maxDriveSpeed = 3;
        public static final double maxTurnSpeed = 3;

        // The threshold of values where the robot will ignore joystick input.
        public static final double deadband = 0.2;

        // Current (amps) limit for drivetrain motors
        public static final int currentLimit = 20;

        // Wheel Circumference
        public static final double wheelCircumference = Units.inchesToMeters(4 * Math.PI);

        // Gear Ratio
        public static final double gearRatio = 10.71;

        // Kinematics
        public static final double trackWidth = Units.inchesToMeters(21);

        // Encoder conversion factors (R to M and RPM to MPS)
        public static final double posConversionFactor = wheelCircumference / gearRatio;
        public static final double velConversionFactor = wheelCircumference / gearRatio / 60;
    }

    public static class LauncherConstants {
        // CAN IDs for motor controllers
        public static final int FeederID = 11;
        public static final int LauncherID = 12;

        // The max speeds for each motor.
        public static final int feedSpeed = 1;
        public static final int launchSpeed = 1;
    }
}
