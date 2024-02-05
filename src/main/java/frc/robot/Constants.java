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

    public static final class DriveConstants {

        public static final double moduleToModuleDistanceMeters = Units.inchesToMeters(12);

        public static final int frontLeftTurnID = 0;
        public static final int frontLeftDriveID = 1;
        public static final int frontLeftEncID = 0;
        public static final double frontLeftOffset = 0.0;

        public static final int backLeftTurnID = 2;
        public static final int backLeftDriveID = 3;
        public static final int backLeftEncID = 1;
        public static final double backLeftOffset = 0.0;

        public static final int frontRightTurnID = 4;
        public static final int frontRightDriveID = 5;
        public static final int frontRightEncID = 2;
        public static final double frontRightOffset = 0.0;

        public static final int backRightTurnID = 6;
        public static final int backRightDriveID = 7;
        public static final int backRightEncID = 3;
        public static final double backRightOffset = 0.0;

    }

    public static final class SwerveConstants {
        public static final double turnkP = 0.0;
        public static final double turnkI = 0.0;
        public static final double turnkD = 0.0;
        public static final double turnkFF = 0.0;

        public static final double drivekP = 0.0;
        public static final double drivekI = 0.0;
        public static final double drivekD = 0.0;
        public static final double drivekFF = 0.0;

        public static final double positionConversionFactor = 1.0;
    }

}
