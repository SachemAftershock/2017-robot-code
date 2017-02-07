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
	 * Instantiates BallShooter object.
	 * 
	 * @param m
	 *            -- shooterMotor controller for flywheel
	 */
	public BallShooter(SpeedController shooterMotor, SpeedController agitatorMotor, Encoder enc) {
		this.shooterMotor = shooterMotor;
		this.agitatorMotor = agitatorMotor;
		this.enc = enc;
		setRPM = 0;
		isUpToSpeed = false;
		isAgitatorOn = false;
	}

	public void disable() {
		PIDLoop.interrupt();
		shooterMotor.set(0);
		agitatorMotor.set(0);
	}

	public void setMotorRPM(double rpm) {
		if (Math.abs(rpm - setRPM) > epsilon) {
			PIDLoop.interrupt();
			PIDLoop = new VelocityPID(rpm, enc, shooterMotor, TUNED_KP, TUNED_KI, TUNED_KD);
		}
	}

	public boolean isUpToSpeed() {
		return isUpToSpeed;
	}

	public void setAgitator(boolean on) {
		isAgitatorOn = on;
	}

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
			double error = desiredRate - inputDevice.get();
			while (!this.isInterrupted()) {
				synchronized (this) {
					integral += error;
					double previousError = error;
					error = desiredRate - inputDevice.get();
					isUpToSpeed = Math.abs(error) < epsilon;
					shooterMotor.set(Kp * error + Ki * integral + Kd * (error - previousError));
					agitatorMotor.set(isAgitatorOn ? 1.0 : 0.0);
				}
			}
		}
	}
}
