package frc.robot.Field;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldElements {

    public static final AprilTag[] tags = new AprilTag[] {
        new AprilTag(1,
            new Pose3d(
                Units.inchesToMeters(593.68),
                Units.inchesToMeters(9.68),
                Units.inchesToMeters(53.38),
                new Rotation3d(0, 0, Math.toRadians(120))
            )
        ),
        new AprilTag(2,
            new Pose3d(
                Units.inchesToMeters(637.21),
                Units.inchesToMeters(34.79),
                Units.inchesToMeters(53.38),
                new Rotation3d(0, 0, Math.toRadians(120))
            )
        ),

        new AprilTag(3,
            new Pose3d(
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(196.17),
                Units.inchesToMeters(57.13),
                new Rotation3d(0, 0, Math.toRadians(180))
            )
        ),
        new AprilTag(4,
            new Pose3d(
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(57.13),
                new Rotation3d(0, 0, Math.toRadians(180))
            )
        ),
        new AprilTag(5,
            new Pose3d(
                Units.inchesToMeters(578.77),
                Units.inchesToMeters(323.00),
                Units.inchesToMeters(53.38),
                new Rotation3d(0, 0, Math.toRadians(270))
            )
        ),
        new AprilTag(6,
            new Pose3d(
                Units.inchesToMeters(72.5),
                Units.inchesToMeters(323.00),
                Units.inchesToMeters(53.38),
                new Rotation3d(0, 0, Math.toRadians(270))
            )
        ),
        new AprilTag(7,
            new Pose3d(
                Units.inchesToMeters(-1.50),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(57.13),
                new Rotation3d(0, 0, Math.toRadians(0))
            )
        ),
        new AprilTag(8,
            new Pose3d(
                Units.inchesToMeters(-1.50),
                Units.inchesToMeters(196.17),
                Units.inchesToMeters(57.13),
                new Rotation3d(0, 0, Math.toRadians(0))
            )
        ),
        new AprilTag(9,
            new Pose3d(
                Units.inchesToMeters(14.02),
                Units.inchesToMeters(34.79),
                Units.inchesToMeters(53.38),
                new Rotation3d(0, 0, Math.toRadians(60))
            )
        ),
        new AprilTag(10,
            new Pose3d(
                Units.inchesToMeters(57.54),
                Units.inchesToMeters(9.68),
                Units.inchesToMeters(53.38),
                new Rotation3d(0, 0, Math.toRadians(60))
            )
        ),
        new AprilTag(11,
            new Pose3d(
                Units.inchesToMeters(468.69),
                Units.inchesToMeters(146.19),
                Units.inchesToMeters(52.00),
                new Rotation3d(0, 0, Math.toRadians(300))
            )
        ),
        new AprilTag(12,
            new Pose3d(
                Units.inchesToMeters(468.69),
                Units.inchesToMeters(177.10),
                Units.inchesToMeters(52.00),
                new Rotation3d(0, 0, Math.toRadians(60))
            )
        ),
        new AprilTag(13,
            new Pose3d(
                Units.inchesToMeters(441.74),
                Units.inchesToMeters(161.62),
                Units.inchesToMeters(52.00),
                new Rotation3d(0, 0, Math.toRadians(180))
            )
        ),
        new AprilTag(14,
            new Pose3d(
                Units.inchesToMeters(209.48),
                Units.inchesToMeters(161.62),
                Units.inchesToMeters(52.00),
                new Rotation3d(0, 0, Math.toRadians(0))
            )
        ),
        new AprilTag(15,
            new Pose3d(
                Units.inchesToMeters(182.73),
                Units.inchesToMeters(177.10),
                Units.inchesToMeters(52.00),
                new Rotation3d(0, 0, Math.toRadians(120))
            )
        ),
        new AprilTag(16,
            new Pose3d(
                Units.inchesToMeters(182.73),
                Units.inchesToMeters(146.19),
                Units.inchesToMeters(52.00),
                new Rotation3d(0, 0, Math.toRadians(240))
            )
        )
    };

    public static final Map<Integer, AprilTag> aprilTags = Map.ofEntries(
        Map.entry(1, tags[0]),
        Map.entry(2, tags[1]),
        Map.entry(3, tags[2]),
        Map.entry(4, tags[3]),
        Map.entry(5, tags[4]),
        Map.entry(6, tags[5]),
        Map.entry(7, tags[6]),
        Map.entry(8, tags[7]),
        Map.entry(9, tags[8]),
        Map.entry(10, tags[9]),
        Map.entry(11, tags[10]),
        Map.entry(12, tags[11]),
        Map.entry(13, tags[12]),
        Map.entry(14, tags[13]),
        Map.entry(15, tags[14]),
        Map.entry(16, tags[15])
    );
}