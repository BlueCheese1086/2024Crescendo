package frc.robot.Watchdog;

import org.littletonrobotics.junction.Logger;

import Util.Interfaces.PowerManaged;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.BatteryConstants;
import frc.robot.Constants.SwerveConstants;

public class Watchdog extends SubsystemBase {
    
    private final PowerDistribution pdh;
    private final Timer timer;

    private final PowerManaged dt;
    private final PowerManaged i;
    private final PowerManaged c;

    private double totalPowerDraw = 0.0;

    public Watchdog(PowerManaged d, PowerManaged i, PowerManaged c) {
        pdh = Robot.pdh;

        dt = d;
        this.i = i;
        this.c = c;

        timer = new Timer();
        timer.start();
    }

    public void periodic() {
        timer.stop();
        totalPowerDraw += pdh.getTotalCurrent() * pdh.getVoltage() * timer.get();
        Logger.recordOutput("Total Wattage Pull", totalPowerDraw);
        SmartDashboard.putNumber("Total Wattage Pull", totalPowerDraw);
        SmartDashboard.putNumber("Current Draw", pdh.getTotalCurrent());
        SmartDashboard.putNumber("Periodic Time", timer.get());

        if (pdh.getTotalCurrent() > getMaximumCurrentDraw()) {
            if (i.getTotalCurrent() > 5.0 || c.getTotalCurrent() > 5.0) {
                dt.setCurrentLimit((int) (getMaximumCurrentDraw() - i.getTotalCurrent() - c.getTotalCurrent()));
            } else {
                dt.setCurrentLimit((int) (dt.getTotalCurrent() - (pdh.getTotalCurrent() - getMaximumCurrentDraw())));
            }
        } else {
            dt.setCurrentLimit(SwerveConstants.DRIVE_CURRENT_LIMIT);
        }

        timer.restart();
    }

    /**
     * @return Returns estimated safe capacity of the battery
     */
    public double getWhRemaining() {
        return BatteryConstants.MAX_Wh - totalPowerDraw;
    }

    /**
     * @return Returns the estimated maximum current draw the battery can handle
     */
    public double getMaximumCurrentDraw() {
        return BatteryConstants.AMP_PULL_ROC * getWhRemaining();
    }

}