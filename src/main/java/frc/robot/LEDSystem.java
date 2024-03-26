package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSystem {
    public void ledinit()
    {
        AddressableLED m_led = new AddressableLED(9);

        AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 0, 0, 255);
        }

        m_led.setData(m_ledBuffer);
        m_led.start();
    }
}
