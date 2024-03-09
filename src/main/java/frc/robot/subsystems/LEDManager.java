package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class LEDManager extends SubsystemBase {
    private final AddressableLED lead;
    private static AddressableLEDBuffer leadbuff;

    private int currLED = 0;
    private int rainbowFirstPixelHue = 0;
    private boolean ledOn;
    private final Timer time = new Timer();

    private static int red = 0;
    private static int green = 0;
    private static int blue = 255;
    private static int custom = 0;

    public LEDManager(boolean on){
        ledOn = on;
        SmartDashboard.putBoolean("LEDS On", ledOn);

        lead = new AddressableLED(0);
        leadbuff = new AddressableLEDBuffer(LEDConstants.ledCount);
        lead.setLength(leadbuff.getLength());

        lead.setData(leadbuff);
        lead.start(); 

        time.reset();
        time.start();
    }

    @Override
    public void periodic() {
        if (ledOn) {
            SmartDashboard.putBoolean("LEDS going", true);
            if (DriverStation.isDisabled()) {
                rainbow();
            } else {
                if(custom == 0){
                     blueCheese();
                }
                if(custom == 1){
                    ledRGB(red, green, blue);
                }
            }
        } else {
            clearLEDs();
        }

        lead.setData(leadbuff);
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
        for (int i = 0; i < LEDConstants.ledCount; i++) {
            if (i < 60) {
                leadbuff.setLED((i+currLED) % LEDConstants.ledCount, Color.kBlue);
            } else {
                leadbuff.setLED((i+currLED) % LEDConstants.ledCount, Color.kYellow);
            }
        }
        currLED++;
    }

    public void clearLEDs() {
        for (int i = 0; i < LEDConstants.ledCount; i++) {
            leadbuff.setRGB(i, 0, 0, 0);
        }
    }

    public void powerLEDs() {
        ledOn = !ledOn;
    }

    public static void ledRGB(int R, int G, int B) {
        red = R;
        green = G;
        blue = B;
        for (int i = 0; i < LEDConstants.ledCount; i++) {
            leadbuff.setRGB(i, R, G, B);
        }
    }

    public static void setCustom(int c){
        custom = c;
    }
    
}
