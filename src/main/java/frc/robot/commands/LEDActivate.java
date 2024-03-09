package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDManager;

public class LEDActivate extends Command {
    private LEDManager led;
    private boolean isEnabled;

    public LEDActivate(LEDManager ledSub) {
        led = ledSub;
        addRequirements(ledSub);
    }

    @Override
    public void initialize() {
        led.powerLEDs();
    }

    @Override
    public void end(boolean interrupted) {
        led.powerLEDs();
    }

}