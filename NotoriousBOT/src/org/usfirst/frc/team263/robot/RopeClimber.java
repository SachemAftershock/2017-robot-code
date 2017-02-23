package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Code to operate rope climber
 * 
 * @author Dan Waxman
 * @version 1.0
 * @since 02-02-17
 */
public class RopeClimber {
	DigitalInput leftLimitSwitch;
	DigitalInput rightLimitSwitch;
	private SpeedController motor;
	private boolean isEnabled;

	/**
	 * Instantiate BallIntake object
	 * 
	 * @param motor
	 *            SpeedController object to control ball intake
	 * @param leftLimitSwitch
	 *            Hard stop limit switch on left side
	 * @param rightLimitSwitch
	 *            Hard stop limit switch on right side
	 */
	public RopeClimber(SpeedController motor, DigitalInput leftLimitSwitch, DigitalInput rightLimitSwitch) {
		this.motor = motor;
		this.leftLimitSwitch = leftLimitSwitch;
		this.rightLimitSwitch = rightLimitSwitch;
		isEnabled = false;
	}

	/**
	 * Updates status of lift motor
	 * 
	 * @param enable
	 *            true if desired to be climbing, false otherwise
	 */
	public void updateEnable(boolean enable) {
		isEnabled = enable;
	}
	
	public void pulse(double speed, long time) {
		motor.set(speed);
		Timer.delay(time / 1000);
		motor.set(0);
	}

	/**
	 * Updates motor output of climber CIM
	 */
	public void run() {
		motor.set(isEnabled /*&& (!leftLimitSwitch.get() && !rightLimitSwitch.get())*/ ? 1.0 : 0.0);
	}
}