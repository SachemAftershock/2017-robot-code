package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
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
	private final double REVOLUTION_CONSTANT = 2400.0, epsilon = 25, TUNED_KP = 0.01, TUNED_KI = 0, TUNED_KD = 0, AGITATOR_SPEED = -0.45;
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
	public BallShooter(SpeedController shooterMotor, SpeedController agitatorMotor, Encoder enc, XboxController controller) {
		this.shooterMotor = shooterMotor;
		this.agitatorMotor = agitatorMotor;
		this.enc = enc;
		setRPM = 0;
		isUpToSpeed = false;
		isAgitatorOn = false;
		this.controller = controller;
		this.enc.setMinRate(0);
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
	
	public void setMotorPower() {
		shooterMotor.set(speed);
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
	
	public void run() {
		agitatorMotor.set(isAgitatorOn ? AGITATOR_SPEED : 0);
		if(controller.getPOV() == 0) {
			speed = 1.0;
		} else if(controller.getPOV() == 90) {
			speed = 0.9;
		} else if(controller.getPOV() == 180) {
			speed = 0.75;
		} else if(controller.getPOV() == 270) {
			speed = 0.85;
		}
	}

	/**
	 * PID Class to control PID on flywheel shooter
	 * 
	 * @author Dan Waxman
	 * @since 02-04-2017
	 * @version 0.1
	 */
	private class VelocityPID extends Thread {
		private double Kp, Ki, Kd, setPoint, previousOmega, error, previousU, integral;
		private Encoder enc;
		private SpeedController motor;
		
		public VelocityPID(double Kp, double Ki, double Kd, double setPoint, Encoder enc, SpeedController output) {
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			this.setPoint = setPoint;
			this.enc = enc;
			this.motor = output;
			previousOmega = enc.getRate();
		}
		
		public void run() {
			//System.out.println("ATETTTTTTTTTTTTTTAS");
			while (PIDGo) {
				synchronized(this) {
					error = setPoint - Math.abs(enc.getRate());
					double u = Kp * error + Ki * integral + Kd * (Math.abs(enc.getRate() - previousOmega)) + previousU;
					System.out.println("error: " + error + " | u: " + u + "| motor: " + motor.get());
					integral += enc.getRate();
					previousU = u;
					previousOmega = enc.getRate();
					motor.set(Math.abs(u) > 1 ? Math.signum(u) : u);
				}	
			}
			PIDGo = false;
			motor.set(0);
			motor = null;
			enc = null;
		}
	}

}