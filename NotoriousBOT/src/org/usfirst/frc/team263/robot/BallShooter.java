package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * Class to contain all methods relating to shooter mechanism on Notorious
 * B.O.T.
 * 
 * @author Dan Waxman
 * @version 0.1
 * @since 01-20-17
 */
public class BallShooter {
	private SpeedController shooterMotor, agitatorMotor;
	private Encoder enc;
	private Thread PIDLoop;
	private final double REVOLUTION_CONSTANT = 600.0, epsilon = 25, TUNED_KP = 1, TUNED_KI = 1, TUNED_KD = 1;
	private double setRPM;
	private volatile boolean isUpToSpeed, isAgitatorOn;

	/**
	 * Instantiates BallShooter object
	 * 
	 * @param shooterMotor
	 *            SpeedController for flywheel
	 * @param agitatorMotor
	 *            SpeedController for agitator in hopper
	 * @param enc
	 *            Rotary encoder on shooter shaft
	 */
	public BallShooter(SpeedController shooterMotor, SpeedController agitatorMotor, Encoder enc) {
		this.shooterMotor = shooterMotor;
		this.agitatorMotor = agitatorMotor;
		this.enc = enc;
		setRPM = 0;
		isUpToSpeed = false;
		isAgitatorOn = false;
	}

	/**
	 * Disables shooter mechanism if currently spinning
	 * 
	 * <p>
	 * Interrupts any PID Controller that may currently be running, then
	 * disables motors.
	 * </p>
	 */
	public void disable() {
		if (PIDLoop != null) {
			PIDLoop.interrupt();
		}
		shooterMotor.set(0);
		agitatorMotor.set(0);
	}

	/**
	 * Creates or maintains rotation rate if within epsilon
	 * 
	 * @param rpm
	 *            Desired rotation rate in rotations per minute
	 */
	public void setMotorRPM(double rpm) {
		if (Math.abs(rpm - setRPM) > epsilon) {
			PIDLoop.interrupt();
			PIDLoop = new VelocityPID(rpm, enc, shooterMotor, TUNED_KP, TUNED_KI, TUNED_KD);
		}
	}

	/**
	 * @return true if current rotation rate is within epsilon, false otherwise.
	 */
	public boolean isUpToSpeed() {
		return isUpToSpeed;
	}

	/**
	 * Sets agitator in hopper to on or off
	 * 
	 * @param enabled
	 *            true if motor should be enabled, false otherwise
	 */
	public void setAgitator(boolean enabled) {
		isAgitatorOn = enabled;
	}

	/**
	 * PID Class to control PID on flywheel shooter
	 * 
	 * @author Dan Waxman
	 * @since 02-04-2017
	 * @version 0.1
	 */
	private class VelocityPID extends Thread {
		private double desiredRate, Kp, Ki, Kd, integral;
		private Encoder inputDevice;
		private SpeedController shooterMotor;
		private final double epsilon;

		public VelocityPID(double desiredRate, Encoder inputDevice, SpeedController shooterMotor, double Kp, double Ki,
				double Kd) {
			this.desiredRate = desiredRate * REVOLUTION_CONSTANT / 60.0;
			this.inputDevice = inputDevice;
			this.shooterMotor = shooterMotor;
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			epsilon = 25;
			integral = 0;
		}

		public void run() {
			double error = desiredRate - inputDevice.getRate();
			while (!this.isInterrupted()) {
				synchronized (this) {
					integral += error;
					double previousError = error;
					error = desiredRate - inputDevice.getRate();
					isUpToSpeed = Math.abs(error) < epsilon;
					shooterMotor.set(Kp * error + Ki * integral + Kd * (error - previousError));
					agitatorMotor.set(isAgitatorOn ? 1.0 : 0.0);
				}
			}
		}
	}
}
