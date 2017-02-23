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
	public static enum GearModes {
		eDown, eUp, eGoingDown, eGoingUp;
	}

	private GearModes state;
	private DigitalInput downwardLimitSwitch, upwardLimitSwitch;
	private boolean desireUp;
	private SpeedController motor;
	private final double MOTOR_SPEED = 0.60, UP_MULTIPLIER = 1.2;

	/**
	 * Initialize GearMechanism object to control Gear subsystem
	 * 
	 * @param motor
	 *            SpeedController object controlling window motor
	 * @param downwardLimitSwitch
	 *            Limit switch on bottom stop of system
	 * @param upwardLimitSwitch
	 *            Limit switch on top of system
	 */
	public GearMechanism(SpeedController motor, DigitalInput downwardLimitSwitch, DigitalInput upwardLimitSwitch) {
		this.motor = motor;
		this.downwardLimitSwitch = downwardLimitSwitch;
		this.upwardLimitSwitch = upwardLimitSwitch;
		state = GearModes.eDown;
		desireUp = false;
	}

	/**
	 * Toggles whether the gear mechanism should currently be going up or down.
	 */
	public void toggleState() {
		desireUp = !desireUp;
	}

	/**
	 * Gets current state of system, used for debugging
	 * 
	 * @return Current GearModes state of system
	 */
	public GearModes getState() {
		return state;
	}

	/**
	 * Contains logic for movement of GearMech gate based upon current set state
	 */
	public void run() {
		if (downwardLimitSwitch.get()) {
			state = GearModes.eDown;
		}
		if (desireUp && !upwardLimitSwitch.get()) {
			state = GearModes.eGoingUp;
		}
		if (upwardLimitSwitch.get()) {
			state = GearModes.eUp;
		}
		if (!desireUp && !downwardLimitSwitch.get()) {
			state = GearModes.eGoingDown;
		}

		switch (state) {
		case eDown:
		case eUp:
			motor.set(0.00);
			break;
		case eGoingUp:
			motor.set(MOTOR_SPEED * UP_MULTIPLIER);
			break;
		case eGoingDown:
			motor.set(-MOTOR_SPEED);
			break;
		default:
			motor.set(0.00);
		}
	}
}