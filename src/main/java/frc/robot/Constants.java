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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    public static final int LeftBackMotor = 2;
    public static final int LeftFrontMotor = 1;
    public static final int RightBackMotor = 4;
    public static final int RightFrontMotor = 3;
    public static final int DrivetrainLimits = 60;
    public static final double DrivetrainSpeed = 1.5;
    public static final double Deadband = 0.2;
  }

  public static class LauncherConstants {
    public static final int UpperMotor = 12;
    public static final int LowerMotor = 11;
    public static final double FlywheelSpeed = 5500;
    public static final double FeedSpeed = 15000;
    public static final double IntakeSpeed = -5500;
  }

  public static class IntakeConstants {

  }

  public static class ClimbConstants {
    
  }
}
