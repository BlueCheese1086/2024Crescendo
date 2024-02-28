package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class ledSubsystem extends SubsystemBase{
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    public ledSubsystem() {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
         led = new AddressableLED(9);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        ledBuffer = new AddressableLEDBuffer(60);
        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();
    }
    
    public void execute() {
         // Fill the buffer with a rainbow
        rainbow();
        // Set the LEDs
        led.setData(ledBuffer);
    }

    public void end(boolean interrupted){}

    public void setRed(){
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 0, 0);
        }
    }

    private void rainbow() {
        // For every pixel
        int rainbowFirstPixelHue = 1;
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
          // Set the value
          ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
      }
}
