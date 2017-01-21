package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Class to contain all methods relating to shooter mechanism on Notorious B.O.T.
 * @author Dan Waxman
 * @version 0.1
 * @since 01-20-17
 */
public class BallShooter {
	SpeedController motor;
	
	// TODO add sensors and semi-autonomous FSMs
	
	/**
	 * Instantiates BallShooter object.
	 * @param m -- motor controller for flywheel
	 */
	public BallShooter(SpeedController motor) {
		this.motor = motor;
	}
	
	/**
	 * Method to directly set speed of flywheel.
	 * @param speed -- speed to set motor controller at ([-1,1])
	 */
	public void setMotorSpeed(double speed) {
		motor.set(speed);
	}
}
