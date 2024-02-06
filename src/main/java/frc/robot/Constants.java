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
        public static final double kPTurn = 0.0;
        public static final double kITurn = 0.0;
        public static final double kDTurn = 0.0;
        public static final double kFFTurn = 0.0;

        public static final double kPDriveVelo = 0.0;
        public static final double kIDriveVelo = 0.0;
        public static final double kDDriveVelo = 0.0;
        public static final double kFFDriveVelo = 0.0;

        public static final double positionConversionFactor = 1.0;
    }

}
