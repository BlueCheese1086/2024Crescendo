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

        public static final double moduleToModuleDistanceMeters = Units.inchesToMeters(19.5);
        public static final double moduleToCenterDistance = Math.sqrt(2*Math.pow(moduleToModuleDistanceMeters/2.0, 2));

        public static final int frontLeftTurnID = 1;
        public static final int frontLeftDriveID = 2;
        public static final int frontLeftEncID = 9;
        public static final double frontLeftOffset = 0.0;

        public static final int backLeftTurnID = 3;
        public static final int backLeftDriveID = 4;
        public static final int backLeftEncID = 1;
        public static final double backLeftOffset = 0.0;

        public static final int frontRightTurnID = 7;
        public static final int frontRightDriveID = 8;
        public static final int frontRightEncID = 8;
        public static final double frontRightOffset = 0.0;

        public static final int backRightTurnID = 5;
        public static final int backRightDriveID = 6;
        public static final int backRightEncID = 0;
        public static final double backRightOffset = 0.0;

        public static final int pigeonID = 2;

        public static final double wheelCircumpherence = Units.inchesToMeters(4.0 * Math.PI);

        public static final double maxWheelVelocity = Units.feetToMeters(19.8);
        public static final double maxRotationalVelocity = maxWheelVelocity/moduleToCenterDistance;

    }

    public static final class SwerveConstants {

        public static final double kPTurn = 1.0;
        public static final double kITurn = 0.0;
        public static final double kDTurn = 0.0;
        public static final double kFFTurn = 0.0;

        public static final double kPDriveVelo = 0.01;
        public static final double kIDriveVelo = 0.0;
        public static final double kDDriveVelo = 0.0;
        public static final double kFFDriveVelo = 0.2;

        public static final double ROC_LIMIT_MULT = 0.1;
        public static final int DRIVE_CURRENT_LIMIT = 80;

        public static final double wheelCircumpherence = Units.inchesToMeters(4.0 * Math.PI);

        public static final double drivePosConversionFactor = wheelCircumpherence / 5.14;
        public static final double steerPosConversionFactor = Math.PI * 2.0 / 12.8;

    }

    public static final class ShooterConstants {

        public static final int frontID = 11;
        public static final int backID = 12;

        public static final double kP = 0.0001;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.01;

    }

    public static final class IntakeConstants {

        public static final int rollerID = 31;
        public static final int angleID = 32;

        public static final double rollersVelocityConversionFactor = 1.0;

        public static final double ROLLERS_CURRENT_LIMIT = 40;
        public static final double ANGLE_CURRENT_LIMIT = 40;

        public static final double angleOffset = 2.0;
        public static final double anglePositionConverstionFactor = Math.PI * 2.0/((48.0/44.0)*(48.0/24.0));

        public static final double STOWED_ANGLE = 2.4;

        public static final double kPRoller = 0.002;
        public static final double kIRoller = 0.0;
        public static final double kDRoller = 0.0;
        public static final double kFFRoller = 1.5;

        public static final double kPAngle = 4.5;
        public static final double kIAngle = 0.0;
        public static final double kDAngle = 0.5;
        public static final double kFFAngle = 4.0;

    }

    public static final class ClimbConstants {
        
        public static final int leftID = 21;
        public static final int rightID = 22;

        public static final double CURRENT_LIMIT = 25;

        public static final double climbGearboxRatio = 25.0;
        public static final double climbConversionFactor = Units.inchesToMeters(1.5*Math.PI) / climbGearboxRatio;

        public static final double maxHeight = Units.inchesToMeters(18);

        public static final double kP = 7.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.0;

    }

    public static final class LEDConstants {

        public static final int PWM_PORT = 0;
        public static final int LENGTH = 255;

    }

    public static final class BatteryConstants {

        public static final double MAX_Wh = 180.0;

        public static final double START_MAX_AMPS = 275.0;
        public static final double END_MAX_AMPS = 150.0;

        public static final double AMP_PULL_ROC = (START_MAX_AMPS - END_MAX_AMPS)/MAX_Wh;

        public static final double motorRpmDropoff = 100;

    }

}
