package frc.robot.SensorsAndFeedback;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDFeedback extends SubsystemBase {

    private final AddressableLED leds;
    private final static AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.LENGTH);

    private static int startingHue = 0;
    private static LEDMode mode = LEDMode.Rainbow;

    public LEDFeedback() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 0, 255);
        }

        leds = new AddressableLED(LEDConstants.PWM_PORT);
        leds.setLength(LEDConstants.LENGTH);
        leds.setData(buffer);
    }

    public void periodic() {
        mode.execute();
    }
    
    private static void rainbow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, startingHue, 255, 255);
        }
        startingHue = (startingHue + 1)%180;
    }

    public void setMode(LEDMode mode) {
		LEDFeedback.mode = mode;
	}

	public enum LEDMode {
		Rainbow(() -> rainbow());

		NullFunction method;

		LEDMode(NullFunction m) {
			method = m;
		}

		public void execute() {
			method.run();
		}
	}

	private interface NullFunction {
		void run();
	}

    
}