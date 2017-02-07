package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * Code for gear mechanism on robot using two limit switches as hard stops.
 * 
 * @author Ryan Bassett
 * @author Dan Waxman
 * @since 02-04-17
 * @version 1.0
 */
public class GearMechanism {
	public enum GearModes {
		eDown, eUp, eGoingDown, eGoingUp;
	}

	private GearModes state;
	private DigitalInput downwardLimitSwitch, upwardLimitSwitch;
	private boolean desireUp;
	private SpeedController motor;
	private final double MOTOR_SPEED = 0.25;

	public GearMechanism(SpeedController motor, DigitalInput downwardLimitSwitch, DigitalInput upwardLimitSwitch) {
		this.motor = motor;
		this.downwardLimitSwitch = downwardLimitSwitch;
		this.upwardLimitSwitch = upwardLimitSwitch;
		state = GearModes.eDown;
		desireUp = false;
	}

	public void toggleState() {
		desireUp = !desireUp;
	}
	
	public GearModes getState() {
		return state;
	}

	public void run() {
		if (!downwardLimitSwitch.get()) {
			state = GearModes.eDown;
		}
		if (desireUp && upwardLimitSwitch.get()) {
			state = GearModes.eGoingUp;
		}
		if (!upwardLimitSwitch.get()) {
			state = GearModes.eUp;
		}
		if (!desireUp && downwardLimitSwitch.get()) {
			state = GearModes.eGoingDown;
		}

		switch (state) {
		case eDown:
		case eUp:
			motor.set(0.00);
			break;
		case eGoingUp:
			motor.set(MOTOR_SPEED);
			break;
		case eGoingDown:
			motor.set(-MOTOR_SPEED);
			break;
		default:
			motor.set(0.00);
		}
	}
}
