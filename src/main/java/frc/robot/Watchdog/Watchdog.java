package frc.robot.Watchdog;

import java.util.Objects;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Watchdog extends SubsystemBase {

    private static Watchdog instance;

    public static Watchdog getInstance() {
        if (Objects.isNull(instance)) {
            instance = new Watchdog();
        }
        return instance;
    }

    private Watchdog() {}

    private double dtMult = 1.0;
    private double inMult = 1.0;
    private double clMult = 1.0;
    private double shMult = 1.0;

    private double drivetrainPriority = 1;
    private double intakePriority = 2;
    private double climbPriority = 4;
    private double shooterPriority = 3;

    private PowerDistribution pdh;

    public void periodic() {

    }

    public void setPDH(PowerDistribution pdh) {
        this.pdh = pdh;
    }

    public double getDrivetrainMultiplier() {
        return dtMult;
    }

    public double getIntakeMultiplier() {
        return inMult;
    }

    public double getClimbMultiplier() {
        return clMult;
    }

    public double getShooterMultiplier() {
        return shMult;
    }

}
