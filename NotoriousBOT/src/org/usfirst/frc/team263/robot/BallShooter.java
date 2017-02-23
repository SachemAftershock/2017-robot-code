package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.XboxController;

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
	private final double epsilon = 25, TUNED_KP = 0.01, TUNED_KI = 0, TUNED_KD = 0, AGITATOR_SPEED = -0.45;
	private double setRPM;
	private XboxController controller;
	private double speed;
	private volatile boolean isUpToSpeed, isAgitatorOn, PIDGo = false;

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
	public BallShooter(SpeedController shooterMotor, SpeedController agitatorMotor, Encoder enc,
			XboxController controller) {
		this.shooterMotor = shooterMotor;
		this.agitatorMotor = agitatorMotor;
		this.controller = controller;
		this.enc = enc;
		this.enc.setMinRate(0);
		setRPM = 0;
		isUpToSpeed = false;
		isAgitatorOn = false;
		speed = 0.0;
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
		PIDGo = false;
		shooterMotor.set(0);
		agitatorMotor.set(0);
	}

	/**
	 * Directly sets the motor power of the ball shooter based on controller
	 * input.
	 */
	public void setMotorPower() {
		if (controller.getPOV() == 0) {
			speed = 0.97;
		} else if (controller.getPOV() == 90) {
			speed = 0.9;
		} else if (controller.getPOV() == 180) {
			speed = 0.75;
		} else if (controller.getPOV() == 270) {
			speed = 0.85;
		}
		shooterMotor.set(speed);
	}

	/**
	 * Directly sets the motor power of the ball shooter.
	 * 
	 * @param power
	 *            Power to set the shooter motor
	 */
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
		if (rpm == 0) {
			PIDGo = false;
		} else if (Math.abs(rpm - setRPM) > epsilon) {
			PIDGo = true;
			PIDLoop = new VelocityPID(TUNED_KP, TUNED_KI, TUNED_KD, rpm, enc, shooterMotor);
			PIDLoop.start();
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
	 * Method to run agitator.
	 */
	public void run() {
		agitatorMotor.set(isAgitatorOn ? AGITATOR_SPEED : 0);
	}

	/**
	 * PID Class to control PID on flywheel shooter.
	 * 
	 * There's probably a better way to do this... I just took a positional PID
	 * and differentiated in terms of time.
	 * 
	 * @author Dan Waxman
	 * @since 02-23-2017
	 * @version 0.1
	 */
	private class VelocityPID extends Thread {
		private double Kp, Ki, Kd, setPoint, previousOmega, error, u, integral;
		private Encoder enc;
		private SpeedController motor;

		/**
		 * Constructor for VPID thread object
		 * @param Kp Tuned proportional gain constant
		 * @param Ki Tuned integral gain constant
		 * @param Kd Tuned derivative gain constant
		 * @param setPoint Desired velocity (in native counts)
		 * @param enc Encoder device for feedback
		 * @param output Device to output to
		 */
		public VelocityPID(double Kp, double Ki, double Kd, double setPoint, Encoder enc, SpeedController output) {
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			this.setPoint = setPoint;
			this.enc = enc;
			this.motor = output;
			previousOmega = enc.getRate();
			u = 0;
		}

		/**
		 * Method for controlling velocity
		 */
		public void run() {
			while (PIDGo) {
				synchronized (this) {
					error = setPoint - Math.abs(enc.getRate());
					u += Kp * error + Ki * integral + Kd * (Math.abs(enc.getRate() - previousOmega));
					integral += enc.getRate();
					previousOmega = enc.getRate();
					motor.set(Math.abs(u) > 1 ? Math.signum(u) : u);
				}
			}
			motor.set(0);
			PIDGo = false;
			motor = null;
			enc = null;
		}
	}

}