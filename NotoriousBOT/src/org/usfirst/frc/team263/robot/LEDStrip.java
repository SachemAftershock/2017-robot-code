package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.I2C;

/**
 * Class to control LED strip on shirt shooter robot over Rio-Arduino
 * communication
 * 
 * @author Rohan Bapat
 * @author Dan Waxman
 * @version 1.3
 * @since 01/29/17
 */
public class LEDStrip {
	private static final int ARDUINO_I2C_ADDRESS = 10;
	private static I2C i2c = new I2C(I2C.Port.kOnboard, ARDUINO_I2C_ADDRESS);
	private static byte[] colorModes = { 'r', 'g', 'b', 'p', 't', 'n', 'a', 'o' };
	public static LEDMode currentMode = LEDMode.eOff;

	/**
	 * Enumeration for different LEDModes
	 */
	public static enum LEDMode {
		eRed, eBlue, ePink, eTeal, eOff, eRainbow, eBlink; 
		//  ^^ took off eGreen bc Vision System light rings are same color
	}

	/**
	 * Send current desired LED Strip mode over I2C to Arduino
	 * 
	 * @param color
	 *            LEDMode enum element to set LEDStrip
	 */
	public static void sendColor(LEDMode color) {
		// Test to make sure color is different than current mode to avoid
		// redundancy
		if (!color.equals(currentMode)) {
			i2c.writeBulk(new byte[] { colorModes[color.ordinal()] });
			currentMode = color;
		}
	}
}