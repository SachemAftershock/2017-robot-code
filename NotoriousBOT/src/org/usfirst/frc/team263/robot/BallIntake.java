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
	 * 
	 * @param motor SpeedController which controls BAG Motor
	 */
	public BallIntake(SpeedController motor) {
		this.motor = motor;
		isEnabled = false;
	}

	/**
	 * Toggles the state of the intake motor (on or off)
	 */
	public void toggleEnable() {
		isEnabled = !isEnabled;
	}

	/**
	 * Sets motor speed of ball intake according to enabled status
	 */
	public void run() {
		motor.set(isEnabled ? 1.0 : 0.0);
	}
}
