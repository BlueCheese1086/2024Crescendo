// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int FRONT_LEFT_ID = 1;
    public static final int BACK_LEFT_ID = 2;
    public static final int FRONT_RIGHT_ID = 3;
    public static final int BACK_RIGHT_ID = 4;

    public static final double DEADBAND = 0.2;
    public static final int DRIVETRAINLIMITS = 60;
    public static final double MAX_ALIGN_SPEED = 0.1;
    public static final double MAX_DRIVE_SPEED = 2; //Meters per second (velcoity)

    public static final double DriveP = 0.25;
    public static final double DriveI = 0;
    public static final double DriveD = 0;
    public static final double DriveFF = 0;

    public static final double wheelDiameter = 0.1524;
    public static final double wheelCircumfrence = wheelDiameter * Math.PI;
    public static final double gearRatio = 10.75;
    public static final double driveRatio = wheelCircumfrence / gearRatio;
  }
  public static class ShooterConstants {
    public static final int UPPER_SHOOTER_ID = 12;
    public static final int LOWER_SHOOTER_ID = 11;
  }
}
