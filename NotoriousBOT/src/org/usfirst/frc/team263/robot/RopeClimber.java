package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;

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
	 * @param motor SpeedController object to control ball intake
	 */
	public RopeClimber(SpeedController motor, DigitalInput leftLimitSwitch, DigitalInput rightLimitSwitch) {
		this.motor = motor;
		this.leftLimitSwitch = leftLimitSwitch;
		this.rightLimitSwitch = rightLimitSwitch;
		isEnabled = false;
	}

	public void updateEnable(boolean enable) {
		isEnabled = enable;
	}

	public void run() {
		motor.set(isEnabled  && (leftLimitSwitch.get() && rightLimitSwitch.get()) ? 1.0 : 0.0);
	}
}
