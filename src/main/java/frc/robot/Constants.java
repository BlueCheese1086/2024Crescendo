// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DriveConstants {
        // Motor IDs
        public static final int frontLeftID = 1;
        public static final int backLeftID = 2;
        public static final int frontRightID = 3;
        public static final int backRightID = 4;

        // Robot specs
        public static final double robotWidth = Units.inchesToMeters(13); // Meters
        public static final double wheelDiameter = Units.inchesToMeters(4); // Meters
        public static final double wheelCircumference = wheelDiameter * Math.PI; // Meters
        public static final double gearRatio = 8.46;

        // Max speeds
        public static final double maxDriveSpeed = 3.5; // Meters / Second
        public static final double maxTurnSpeed = maxDriveSpeed / (robotWidth / 2); // Meters / Second

        // Conversion factors
        public static final double posConversionFactor = wheelCircumference / gearRatio;
        public static final double velConversionFactor = wheelCircumference / gearRatio / 60;

        // PIDFF values
        public static final double leftP   = 0.0;
        public static final double leftI   = 0.0;
        public static final double leftD   = 0.0;
        public static final double leftFF  = 0.0;
        public static final double rightP  = 0.0;
        public static final double rightI  = 0.0;
        public static final double rightD  = 0.0;
        public static final double rightFF = 0.0;
    }

    public static class ShooterConstants {
        // Motor IDs
        public static final int feedID = 11;
        public static final int launchID = 12;

        // Max speeds
        public static final double maxFeedSpeed = 5676; // Rotations / Minute
        public static final double maxLaunchSpeed = 5676; // Rotations / Minute

        // PIDFF values
        public static final double launchP  = 0.0;
        public static final double launchI  = 0.0;
        public static final double launchD  = 0.0;
        public static final double launchFF = 0.0;
        public static final double feedP    = 0.0;
        public static final double feedI    = 0.0;
        public static final double feedD    = 0.0;
        public static final double feedFF   = 0.0;
    }
}