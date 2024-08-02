// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double loopPeriodSecs = 0.02;

    public static final double LOOP_TIME = 0.13;
    public static final double ROBOT_MASS = 49.8951607;
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final boolean isTuning = true;

    public static class RobotMap {
        public static class Drive {
            public static final int frontLeftDrive = 1;
            public static final int frontLeftTurn = 2;
            public static final int frontRightDrive = 3;
            public static final int frontRightTurn = 4;
            public static final int backLeftDrive = 5;
            public static final int backLeftTurn = 6;
            public static final int backRightDrive = 7;
            public static final int backRightTurn = 8;

            public static final boolean frontLeftDriveInvert = true;
            public static final boolean frontLeftTurnInvert = false;
            public static final boolean frontRightDriveInvert = false;
            public static final boolean frontRightTurnInvert = false;
            public static final boolean backLeftDriveInvert = false;
            public static final boolean backLeftTurnInvert = false;
            public static final boolean backRightDriveInvert = true;
            public static final boolean backRightTurnInvert = false;

            public static final int frontLeftEncoder = 2;
            public static final int frontRightEncoder = 3;
            public static final int backLeftEncoder = 1;
            public static final int backRightEncoder = 0;

            public static final double frontLeftOffset = 0.0;
            public static final double frontRightOffset = 0.0;
            public static final double backLeftOffset = -2.035203218460083;
            public static final double backRightOffset = -3.0376768112182617;

            public static final int gyro = 10;
        }

        public static class Intake {
            public static final int pivot = 11;
            public static final int roller = 12;
            public static final int feeder = 21;
        }

        public static class Shooter {
            public static final int pivot = 22;
            public static final int left = 23;
            public static final int right = 24;

            public static final int feederBeambreak = 1;
            public static final int shooterBeambreak = 0;
        }

        public static class Climber {
            public static final int climber = 31;
        }

    }

    public static class ControlConstants {
        public static final double deadband = 0.01;
    }

    public static class DriveConstants {
        public static final boolean wheelsStraight = false;

        public static final double trackWidth = Units.inchesToMeters(19.5);
        public static final double wheelRadius = Units.inchesToMeters(2);

        public static final double driveRatio = 5.14;
        public static final double driveMOI = 0.025;
        public static final double turnRatio = 12.8;
        public static final double turnMOI = 0.004;

        public static final double driveConversion = (driveRatio) * (1.0 / (wheelRadius * 2 * Math.PI));
        public static final double turnConversion = 2 * Math.PI / turnRatio;
        public static final double turnVelocityConversion = turnConversion / 60;

        public static final int driveSupplyCurrent = 70; // 70
        public static final int driveStatorCurrent = 120; // 120
        public static final int turnCurrent = 30; // 30

        public static final double odometeryFrequency = 250;
        public static final double updateFrequency = 100;

        // public static final double maxLinearVelocity = Units.feetToMeters(20.4);
        public static final double maxLinearVelocity = Units.feetToMeters(20.4);
        public static final double maxLinearAccel = 8.0;

        public static final double maxAngularVelocity = maxLinearVelocity
                / (Math.hypot(trackWidth / 2.0, trackWidth / 2.0));
        public static final double maxAngularAccel = maxLinearAccel / (Math.hypot(trackWidth / 2.0, trackWidth / 2.0));

        public static double kPDriveReal = 2.0;
        public static double kDDriveReal = 0.2;
        public static double kSDriveReal = 0.04;
        public static double kVDriveReal = 1.93;
        public static double kADriveReal = 0.25;

        public static double kPTurnReal = 1; // 1.5?
        public static double kDTurnReal = 0.0;

        public static double kPDriveSim = 0.3;
        public static double kDDriveSim = 0.0;
        public static double kSDriveSim = 0.0;
        public static double kVDriveSim = 2.0;
        public static double kADriveSim = 0.0;

        public static double kPTurnSim = 100.0;
        public static double kDTurnSim = 0.0;

        public static double kPDriveReplay = 0.0;
        public static double kDDriveReplay = 0.0;
        public static double kSDriveReplay = 0.0;
        public static double kVDriveReplay = 0.0;
        public static double kADriveReplay = 0.0;

        public static double kPTurnReplay = 0.0;
        public static double kDTurnReplay = 0.0;
    }

    public static class ClimberConstants {
        public static double gearRatio = 45;
        public static double spoolRadius = Units.inchesToMeters(.75);
        public static double encoderConversion = 2 * spoolRadius * Math.PI / gearRatio;
        public static double width = Units.inchesToMeters(2.0);

        public static double minHeight = 0.0;
        public static double maxHeight = Units.inchesToMeters(18);

        public static double maxVelocity = Units.inchesToMeters(17);
        public static double maxAccel = Units.inchesToMeters(180);

        public static double kPSim = 20;
        public static double kISim = 0.0;
        public static double kDSim = 0.0;

        public static double kPReal = 7.5;
        public static double kIReal = 0.0;
        public static double kDReal = 0.0;

        public static double kPReplay = 0.0;
        public static double kIReplay = 0.0;
        public static double kDReplay = 0.0;

        public static double kFFSim = 0.0;
        public static double kFFReal = 0.0;
        public static double kFFReplay = 0.0;

        public static double kSSim = 0.0;
        public static double kGSim = 0.0;
        public static double kVSim = 0.0;
        public static double kASim = 0.0;
    }

    public static class IntakeConstants {
        public static final double pivotRatio = 25.0 * 48.0 / 44.0 * 48.0 / 24.0;
        public static final double pivotMOI = 0.0022842632;
        public static final double pivotLength = Units.inchesToMeters(9.41628595);

        public static final double pivotOffset = 1.1;
        public static final boolean pivotInvert = true;

        public static final double maxPivotVelocity = 10.6081112;
        public static final double maxPivotAccel = 5;

        public static final double pivotAbsConversion = Math.PI * 2.0 / ((48.0 / 44.0) * (48.0 / 24.0));
        public static final double pivotEncConversion = 2.0 * Math.PI / pivotRatio;

        public static final double rollerMOI = 0.011328;

        public static final double up = .2;
        public static final double down = 2.1;
        public static final double simOffset = 1.27838411;

        public static final int pivotCurrentLimit = 20;
        public static final int rollerCurrentLimit = 80;

        public static final double kGPivot = 0.5;
        public static final double kVPivot = 1.06;
        public static final double kAPivot = 0.02;

        public static final double kVRoller = 0.0029;
        public static final double kARoller = 0;

        public static double kPPivotReal = .7;

        public static double kPRollerReal = 0.0000;
        public static double kSRollerReal = 0.0;

        public static double kPPivotSim = 1.25;

        public static double kPRollerSim = 0.0005;
        public static double kSRollerSim = 0.0;

        public static double kPPivotReplay = 0.3;

        public static double kPRollerReplay = 10;
        public static double kSRollerReplay = 0.0;
    }

    public static class FeederConstants {
        public static final double ratio = 30.0 / 18.0;
        public static final double MOI = 0.0109330333;

        public static final int currentLimit = 30;

        public static double kPReal = 0.0001;
        public static double kVReal = 0.0;

        public static final double kPSim = 0.1;
        public static final double kVSim = 0.12;

        public static final double kPReplay = 0.0;
        public static final double kVReplay = 0.0;
    }

    public static class ShooterConstants {
        public static final double pivotRatio = 48.0;
        public static final double pivotMOI = 0.0022842632;
        public static final double pivotLength = Units.inchesToMeters(7.01793315);

        public static final double maxPivotVelocity = 10.5819313;
        public static final double maxPivotAccel = 5;

        public static final double pivotAbsConversion = Math.PI * 2.0 / (33.0 / 34.0);
        public static final double pivotEncConversion = 2.0 * Math.PI / pivotRatio;
        public static final double pivotOffset = 3.5780138;
        public static final double simOffset = 0.0;

        public static final double down = 0.191986218;
        public static final double up = 1.19;

        public static final double shooterMOI = 0.00920287973;

        public static final int pivotCurrentLimit = 40;

        public static final double kGPivot = .9;
        public static final double kVPivot = 0;
        public static final double kAPivot = 0.00;

        public static final double kVShooter = 0.38;
        public static final double kAShooter = 0.25;

        public static final double kPShooterReal = 0.5;
        public static final double kSShooterReal = 0.5;

        public static final double kPPivotReal = 0.0;
        public static final double kPPivotSim = 100.0;

        public static final double kPShooterSim = 0.5;
        public static final double kSShooterSim = 0.5;

        public static final double kPPivotReplay = 0.0;

        public static final double kPShooterReplay = 0.0;
        public static final double kSShooterReplay = 0.0;
    }

    public static class SimConstants {
        public static final double loopTime = 0.02;
    }
}
