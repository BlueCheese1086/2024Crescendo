package frc.robot.LEDManager;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDManager extends SubsystemBase {
    private final AddressableLED lead;
    private final AddressableLEDBuffer leadbuff;

    private int rainbowFirstPixelHue = 0;
    private boolean ledOn;
    private final Timer time = new Timer();

    public LEDManager(boolean on){
        ledOn = on;

        lead = new AddressableLED(0);
        leadbuff = new AddressableLEDBuffer(120);
        lead.setLength(leadbuff.getLength());

        lead.setData(leadbuff);
        lead.start(); 
    }

    @Override
    public void periodic() {
        if (ledOn) {
            if (DriverStation.isDisabled()) {
                rainbow();
            } else {
                blueCheese();
            }
        }
    }

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < leadbuff.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (rainbowFirstPixelHue + (i * 180 / leadbuff.getLength())) % 180;
          // Set the value
          leadbuff.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    private void blueCheese() {
        Color color = (time.get() * 10 % 10) % 5 < 2.5 ? Color.kBlue : Color.kYellow;
        for (int i = 0; i < 120; i++) {
            leadbuff.setLED(i, color);
        }
    }
    
}
