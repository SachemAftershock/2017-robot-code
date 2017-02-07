package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Code to operate ball intake
 * 
 * @author Dan Waxman
 * @version 1.0
 * @since 02-02-17
 */
public class BallIntake {
	private SpeedController motor;
	private boolean isEnabled;
	
	/**
	 * Instantiate BallIntake object
	 * @param motor -- motor 
	 */
	public BallIntake(SpeedController motor) {
		this.motor = motor;
		isEnabled = false;
	}
	
	public void toggleEnable() {
		isEnabled = !isEnabled;
	}
	
	public void run() {
		motor.set(isEnabled ? 1.0 : 0.0);
	}
}
