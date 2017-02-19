package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
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
	private final double REVOLUTION_CONSTANT = 600.0, epsilon = 25, TUNED_KP = Math.pow(10, -3), TUNED_KI = 0, TUNED_KD = Math.pow(10, -1), AGITATOR_SPEED = -0.45;
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
	
	public void setMotorPower(double power) {
		shooterMotor.set(power);
	}

	/**
	 * Creates or maintains rotation rate if within epsilon
	 * 
	 * @param rpm
	 *            Desired rotation rate in rotations per minute
	 */
	public void setMotorRPM(double rpm) {
		if (rpm == 0.0) {
			if (PIDLoop != null) {
				PIDLoop.interrupt();
			}
		} else if (Math.abs(rpm - setRPM) > epsilon) {
			if (PIDLoop != null) {
				PIDLoop.interrupt();
			}
			PIDLoop = new VelocityPID(rpm, enc, shooterMotor, TUNED_KP, TUNED_KI, TUNED_KD);
			PIDLoop.run();
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
	
	public void run() {
		agitatorMotor.set(isAgitatorOn ? AGITATOR_SPEED : 0);
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
			
			inputDevice.setPIDSourceType(PIDSourceType.kRate);
		}

		public void run() {
			double error = desiredRate - inputDevice.pidGet();
			while (!this.isInterrupted()) {
				synchronized (this) {
					integral += error;
					double previousError = error;
					error = desiredRate - inputDevice.pidGet();
					isUpToSpeed = Math.abs(error) < epsilon;
					double u = -(Kp * error + Ki * integral + Kd * (error - previousError));
					System.out.println("Error: " + error + " | u: " + u + " | y: " + inputDevice.pidGet());
					shooterMotor.set(Math.abs(u) > 1 ? Math.signum(u) : u);
					agitatorMotor.set(isAgitatorOn ? AGITATOR_SPEED: 0.0);
				}
			}
			System.out.println("test");
			disable();
			shooterMotor = null;
			inputDevice = null;
		}
	}
}