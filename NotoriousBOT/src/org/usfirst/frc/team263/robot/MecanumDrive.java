package org.usfirst.frc.team263.robot;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Drivebase code for a mecanum drivebase. Supports field-centric and
 * non-oriented driving methods.
 * 
 * @version 1.1
 * @author Dan Waxman
 * @since 01-20-17
 */
public class MecanumDrive {
	private Thread rotationThread;
	private SpeedController mFrontRight, mBackRight, mFrontLeft, mBackLeft;
	private double kd;
	public volatile boolean autoMovement;
	public double speed;
	private AHRS mGyro;
	private boolean clientCameraToggled;
	private final double TUNED_KP = 0.0076, TUNED_KI = 0.0025, TUNED_KD = -0.0025, TUNED_KF = 0.02, ROTATION_CONSTANT = 0.6;

	// private final double TUNED_KP = 0.1, TUNED_KI = Math.pow(10, -5),
	// TUNED_KD = 0.3, ROTATION_CONSTANT = 0.6;
	public enum PIDModes {
		eRotate, eLinearX, eLinearY;
	}

	/**
	 * Construct instance of drivebase code with appropriate motor controllers
	 * to be addressed.
	 * 
	 * @param frontRight
	 *            front right wheel motor controller
	 * @param backRight
	 *            back right wheel motor controller
	 * @param frontLeft
	 *            front left wheel motor controller
	 * @param backLeft
	 *            back left wheel motor controller
	 * @param gyro
	 *            AHRS device to read yaw from
	 * @param driftConstant
	 *            proportionality constant for angle drift of drivebase
	 */
	public MecanumDrive(SpeedController frontRight, SpeedController backRight, SpeedController frontLeft,
			SpeedController backLeft, AHRS gyro, double driftConstant) {
		mFrontRight = frontRight;
		mBackRight = backRight;
		mFrontLeft = frontLeft;
		mBackLeft = backLeft;
		kd = driftConstant;
		autoMovement = false;
		mGyro = gyro;
		clientCameraToggled = false;
		speed = 0.69;
	}

	/**
	 * Construct instance of drivebase code with appropriate motor controllers
	 * to be addressed.
	 * 
	 * @param frontRight
	 *            front right wheel motor controller
	 * @param backRight
	 *            back right wheel motor controller
	 * @param frontLeft
	 *            front left wheel motor controller
	 * @param backLeft
	 *            back left wheel motor controller
	 * @param gyro
	 *            AHRS device to read yaw from
	 */
	public MecanumDrive(SpeedController frontRight, SpeedController backRight, SpeedController frontLeft,
			SpeedController backLeft, AHRS gyro) {
		this(frontRight, backRight, frontLeft, backLeft, gyro, 0.005);
	}

	/**
	 * Method to drive a mecanum drivebase off of Xbox Controller input with
	 * field-centric controls and rotational closed system feedback.
	 * 
	 * <p>
	 * This method is field oriented, meaning that a control direction is
	 * absolute to the field's Cartesian plane.
	 * </p>
	 * <p>
	 * Also offers closed loop rotational feedback system.
	 * </p>
	 * <p>
	 * Refer to <code>drive(Xbox controller)</code> for non-field oriented
	 * controls.
	 * </p>
	 * s
	 * 
	 * @param controller
	 *            Xbox controller to input drive controls
	 * @param fieldCentric
	 *            true if field centric controls, false otherwise.
	 */
	public void drive(XboxController controller, boolean fieldCentric) {
		if (!autoMovement) {
			// Get controller inputs with artificial deadband.
			// y-axis is negated in order to make driving more intuitive.
			double x = deadband(controller.getRawAxis(0), 0.1);
			double y = deadband(-controller.getRawAxis(1), 0.1);
			double r = deadband(controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft), 0.1);

			if (controller.getBumper(Hand.kRight)) {
				x = 0;
			}

			// Drift correction using proportional gain
			if (r == 0 && Math.abs(x) + Math.abs(y) > 0) {
				r = kd * mGyro.getRate();
			}

			if (fieldCentric) {
				// Perform vector rotation in R^2 by theta degrees
				double theta = mGyro.getYaw();
				double sinT = Math.sin(Math.toRadians(theta));
				double cosT = Math.cos(Math.toRadians(theta));
				double yPrime = x * sinT + y * cosT;
				x = x * cosT - y * sinT;
				y = yPrime;
			}

			// Speeds = {fr, br, fl, bl} operations for each wheel speed.
			// Speeds are then normalized to make sure the robot drives
			// correctly.
			double[] speeds = { -x + y - r * ROTATION_CONSTANT, x + y - r * ROTATION_CONSTANT,
					x + y + r * ROTATION_CONSTANT, -x + y + r * ROTATION_CONSTANT };
			normalize(speeds);

			double throttleMultiplier = controller.getBumper(Hand.kLeft) ? 0.5 : 1.0;

			mFrontRight.set(speeds[0] * throttleMultiplier);
			mBackRight.set(speeds[1] * throttleMultiplier);
			mFrontLeft.set(speeds[2] * throttleMultiplier);
			mBackLeft.set(speeds[3] * throttleMultiplier);

			if (controller.getXButton()) {
				synchronized (this) {
					autoMovement = true;
					rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, TUNED_KF, mGyro, -90,
							new SpeedController[] { mFrontRight, mBackRight, mFrontLeft, mBackLeft },
							new double[] { 1, 1, -1, -1 });
					rotationThread.start();
				}
			} else if (controller.getBButton()) {
				synchronized (this) {
					autoMovement = true;
					rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, TUNED_KF, mGyro, 90,
							new SpeedController[] { mFrontRight, mBackRight, mFrontLeft, mBackLeft },
							new double[] { 1, 1, -1, -1 });
					rotationThread.start();
				}
			} else if (controller.getYButton()) {
				synchronized (this) {
					autoMovement = true;
					rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, TUNED_KF, mGyro, 0,
							new SpeedController[] { mFrontRight, mBackRight, mFrontLeft, mBackLeft },
							new double[] { 1, 1, -1, -1 });
					rotationThread.start();
				}
			}
		} else if (controller.getAButton()) {
			autoMovement = false;
		} 
		/*
		 * if(controller.getStartButton() && clientCameraToggled) {
		 * CameraCoprocessor.toggleClientCamera(); }
		 * 
		 * clientCameraToggled = controller.getStartButton();
		 * CameraCoprocessor.setClientCamera();
		 */
	}

	/**
	 * Normalizes an array of values to [-1,1] scale.
	 * 
	 * <p>
	 * Optimal for motor normalization to create ranged speed control values
	 * </p>
	 * 
	 * @param array
	 *            array of values to normalize from [-1,1] scale
	 */
	private void normalize(double[] array) {
		boolean normFlag = false;
		double maxValue = array[0];

		for (double value : array) {
			if (Math.abs(value) > maxValue) {
				maxValue = Math.abs(value);
				normFlag = maxValue > 1;
			}
		}

		if (normFlag) {
			for (int i = 0; i < array.length; i++) {
				array[i] /= maxValue;
			}
		}
	}

	public void strafe(double speed, long time) {
		mFrontRight.set(-speed);
		mFrontLeft.set(speed);
		mBackRight.set(speed);
		mBackLeft.set(-speed);
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		mFrontRight.set(0);
		mFrontLeft.set(0);
		mBackRight.set(0);
		mBackLeft.set(0);
	}

	public void rotate(double speed, long time) {
		mFrontRight.set(-speed);
		mFrontLeft.set(speed);
		mBackRight.set(-speed);
		mBackLeft.set(speed);
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		mFrontRight.set(0);
		mFrontLeft.set(0);
		mBackRight.set(0);
		mBackLeft.set(0);
	}
	
	public void autoRotate(double theta) {
		synchronized (this) {
			autoMovement = true;
			rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, TUNED_KF, mGyro, theta,
					new SpeedController[] { mFrontRight, mBackRight, mFrontLeft, mBackLeft },
					new double[] { 1, 1, -1, -1 });
			rotationThread.start();
		}
	}

	/**
	 * Creates artificial absolute deadband on values.
	 * 
	 * @param value
	 *            value to create deadband on
	 * @param deadband
	 *            minimum number that <code>abs(value)</code> must exceed
	 * @return value if <code>abs(value)</code> is greater than deadband, 0
	 *         otherwise
	 */
	private double deadband(double value, double deadband) {
		return Math.abs(value) > deadband ? value : 0.0;
	}

	/**
	 * PID Class for auto base rotation
	 * 
	 * @author Dan Waxman
	 * @version 0.1
	 * @since 01/26/17
	 */
	private class PIDController extends Thread {
		private double kp, ki, kd, kf, setPoint, error, previousError, previousOmega, alpha, integral;
		private AHRS inputDevice;
		private SpeedController[] motors;
		private double[] multipliers;
		private final double epsilon = 5;
		
		public PIDController(double Kp, double Ki, double Kd, double Kf, AHRS gyro, double setPoint, SpeedController[] motors, double[] multipliers) {
			this.kp = Kp;
			this.ki = Ki;
			this.kd = Kd;
			this.kf = Kf;
			this.setPoint = setPoint;
			this.motors = motors; 
			this.multipliers = multipliers;
			this.inputDevice = gyro;
		}
		
		public void run() {
			error = rotationalError(inputDevice.getYaw(), setPoint);
			previousError = error;
			alpha = 0;
			previousOmega = inputDevice.getRate();
			while (Math.abs(error) > epsilon && autoMovement) {
				synchronized(this) {
					integral += error;
					alpha = inputDevice.getRate() - previousOmega;
					previousOmega = inputDevice.getRate();
					double iTerm = Math.abs(previousError - error) < 2 ? ki * integral : 0;
					double u = kp * error + iTerm + kd * ( 	previousError - error) - kf * Math.abs(alpha);
					//if (Math.abs(iTerm) > 0 && Math.)
					double[] motorSpeeds = new double[motors.length];
					for (int i = 0; i < motors.length; i++) {
						motorSpeeds[i] = u * multipliers[i];
					}
					normalize(motorSpeeds);
					for (int i = 0; i < motors.length; i++) {
						motorSpeeds[i] *= 0.6;
					}
					for (int i = 0; i < motors.length; i++) {
						motors[i].set(motorSpeeds[i]);
					}
					error = rotationalError(inputDevice.getYaw(), setPoint);
					if (Math.abs(error) > epsilon) {
						try {
							Thread.sleep(500);
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
						error = rotationalError(inputDevice.getYaw(), setPoint);
					}
				}
			}
			motors = null;
			inputDevice = null;
			autoMovement = false;
		}
		
		private double rotationalError(double alpha, double beta) {
			double ret = alpha - beta;
			if (ret < -180) {
				ret += 360;
			}
			if (ret > 180) {
				ret -= 360;
			}
			return ret;
		}
	}

	public void forward(double d, int i) {
		mFrontLeft.set(d);
		mFrontRight.set(d);
		mBackLeft.set(d);
		mBackRight.set(d);
		try {
			Thread.sleep(i);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			// e.printStackTrace();
		}
		mFrontLeft.set(0);
		mFrontRight.set(0);
		mBackLeft.set(0);
		mBackRight.set(0);
	}
}