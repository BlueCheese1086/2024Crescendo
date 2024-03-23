package frc.robot.SensorsAndFeedback;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDFeedback extends SubsystemBase {

    private final AddressableLED leds;
    private final static AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.LENGTH);

    private static int startingHue = 0;
    private static int bootingLed = 1;
    private static Timer ledTimer = new Timer();
    private static LEDMode mode = LEDMode.Bootup;

    private static double cheeseDex = 0.0;
	private static double cheeseMult = 1.0;

    public LEDFeedback() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }

        leds = new AddressableLED(LEDConstants.PWM_PORT);
        leds.setLength(LEDConstants.LENGTH);
        leds.setData(buffer);
        leds.start();
        ledTimer.start();
    }

    public void periodic() {
        mode.execute();
        leds.setData(buffer);
    }
    
    private static void bootup() {
        // System.out.println(buffer.getLength());
        for (int i = 0; i < bootingLed; i++) {
            buffer.setRGB(i, 0, 0, 255);
            if (bootingLed < buffer.getLength() && ledTimer.get() > 0.020) {
                bootingLed+=1;
                ledTimer.restart();
            } else if (bootingLed >= buffer.getLength()) {
                mode = LEDMode.Cheese;
                ledTimer.stop();
            }
        }
    }

    private static void cheese() {
		if (cheeseDex > 1.0) {
			cheeseMult = -1;
			cheeseDex = 1;
		} else if (cheeseDex < 0.0) {
			cheeseMult = 1;
			cheeseDex = 0;
		}
		SmartDashboard.putNumber("/LEDS/DEX", cheeseDex);
		for (int i = 0; i < buffer.getLength(); i++) {
			buffer.setRGB(
					i,
					(int) (Color.kYellow.red * cheeseDex * 255
							+ (1 - cheeseDex) * Color.kBlue.red * 255),
					(int) (Color.kYellow.green * cheeseDex * 255
							+ (1 - cheeseDex) * Color.kBlue.green * 255),
					(int) (Color.kYellow.blue * cheeseDex * 255
							+ (1 - cheeseDex) * Color.kBlue.blue * 255));
		}
		cheeseDex = cheeseDex + (1.0 / 250.0 * cheeseMult);
		// System.out.println(cheeseDex);
	}

    private static void blank() {
        setColor(0, 0, 0);
    }

    private static void blue() {
        setColor(0, 0, 255);
    }

    private static void rainbow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, startingHue, 255, 255);
        }
        startingHue = (startingHue + 1)%180;
    }

    private static void red() {
        setColor(255, 0, 0);
    }

    private static void setColor(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    public void setMode(LEDMode mode) {
		LEDFeedback.mode = mode;
	}

	public enum LEDMode {
		Rainbow(() -> rainbow()),
        Bootup(() -> bootup()),
        Red(() -> red()),
        Blue(() -> blue()),
        Blank(() -> blank()),
        Cheese(() -> cheese());

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