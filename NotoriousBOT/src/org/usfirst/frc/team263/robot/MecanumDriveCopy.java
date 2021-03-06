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
public class MecanumDriveCopy {
	private Thread rotationThread;
	private SpeedController mFrontRight, mBackRight, mFrontLeft, mBackLeft;
	private double kd;
	public volatile boolean autoMovement;
	private AHRS mGyro;
	private boolean clientCameraToggled;
	private final double TUNED_KP = 0.013 * 1.1, TUNED_KI = 0.00005 * 1.1, TUNED_KD = 0.006 * 1.1, ROTATION_CONSTANT = 0.6;

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
	public MecanumDriveCopy(SpeedController frontRight, SpeedController backRight, SpeedController frontLeft,
			SpeedController backLeft, AHRS gyro, double driftConstant) {
		mFrontRight = frontRight;
		mBackRight = backRight;
		mFrontLeft = frontLeft;
		mBackLeft = backLeft;
		kd = driftConstant;
		autoMovement = false;
		mGyro = gyro;
		clientCameraToggled = false;
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
	public MecanumDriveCopy(SpeedController frontRight, SpeedController backRight, SpeedController frontLeft,
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
					rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, mGyro, -90,
							new SpeedController[] { mFrontRight, mBackRight, mFrontLeft, mBackLeft },
							new double[] { 1, 1, -1, -1 });
					rotationThread.start();
				}
			} else if (controller.getBButton()) {
				synchronized (this) {
					autoMovement = true;
					rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, mGyro, 90,
							new SpeedController[] { mFrontRight, mBackRight, mFrontLeft, mBackLeft },
							new double[] { 1, 1, -1, -1 });
					rotationThread.start();
				}
			} else if (controller.getYButton()) {
				synchronized (this) {
					autoMovement = true;
					rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, mGyro, 0,
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

	public void autoStrafe(double distance) {
		synchronized (this) {
			autoMovement = true;
			rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, mGyro, distance,
					new SpeedController[] { mFrontRight, mBackRight, mFrontLeft, mBackLeft },
					new double[] { -1, 1, 1, -1 }, PIDModes.eLinearX);
			rotationThread.start();
		}
	}

	public void autoDrive(double distance) {
		synchronized (this) {
			autoMovement = true;
			rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, mGyro, distance,
					new SpeedController[] { mFrontRight, mBackRight, mFrontLeft, mBackLeft },
					new double[] { 1, 1, 1, 1 }, PIDModes.eLinearY);
			rotationThread.start();
		}
	}

	public void autoRotate(double theta) {
		synchronized (this) {
			autoMovement = true;
			rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, mGyro, theta,
					new SpeedController[] { mFrontRight, mBackRight, mFrontLeft, mBackLeft },
					new double[] { 1, 1, -1, -1 }, PIDModes.eRotate);
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
		private double Kp, Ki, Kd, setPoint, integral, previousError, epsilon, initialDisplacement;
		private AHRS inputDevice;
		private SpeedController[] motors;
		private double[] multipliers;
		private PIDModes mode;

		public PIDController(double Kp, double Ki, double Kd, AHRS inputDevice, double setPoint,
				SpeedController[] motors, double[] multipliers) {
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			this.inputDevice = inputDevice;
			this.setPoint = setPoint;
			this.motors = motors;
			this.multipliers = multipliers;
			this.mode = PIDModes.eRotate;
			integral = 0;
			previousError = setPoint - rotateError(inputDevice.getYaw(), setPoint);
			epsilon = 5;
		}

		public PIDController(double Kp, double Ki, double Kd, AHRS inputDevice, double setPoint,
				SpeedController[] motors, double[] multipliers, PIDModes mode) {
			this.Kp = Kp;
			this.Ki = Ki;
			this.Kd = Kd;
			this.inputDevice = inputDevice;
			this.setPoint = setPoint;
			this.motors = motors;
			this.multipliers = multipliers;
			this.mode = mode;
			integral = 0;
			epsilon = 5;
			if (mode.equals(PIDModes.eRotate)) {
				previousError = rotateError(inputDevice.getYaw(), setPoint);
			} else if (mode.equals(PIDModes.eLinearX)) {
				initialDisplacement = inputDevice.getDisplacementX();
				previousError = linearError(setPoint, inputDevice.getDisplacementX() - initialDisplacement);
			} else {
				initialDisplacement = inputDevice.getDisplacementY();
				previousError = linearError(setPoint, inputDevice.getDisplacementY() - initialDisplacement);
			}
		}

		public void run() {
			Timer t = new Timer();
			double error;
			if (mode.equals(PIDModes.eRotate)) {
				error = rotateError(inputDevice.getYaw(), setPoint);
			} else if (mode.equals(PIDModes.eLinearX)) {
				error = linearError(setPoint, inputDevice.getDisplacementX() - initialDisplacement);
			} else {
				error = linearError(setPoint, inputDevice.getDisplacementY() - initialDisplacement);
			}
			synchronized (this) {
				while (Math.abs(error) > epsilon && autoMovement) {
					if (mode.equals(PIDModes.eRotate)) {
						error = rotateError(inputDevice.getYaw(), setPoint);
					} else if (mode.equals(PIDModes.eLinearX)) {
						error = linearError(setPoint, inputDevice.getDisplacementX() - initialDisplacement);
					} else {
						error = linearError(setPoint, inputDevice.getDisplacementY() - initialDisplacement);
					}
					integral += error;
					double iTerm = Math.abs(previousError - error) > 2 ? 0 : Ki * integral;
					double u = Kp * error + iTerm + Kd * (error - previousError);
					double[] motorSpeeds = new double[motors.length];
					if (Math.abs(previousError - error) < 2) {
						System.out.println(u);
					}
					for (int i = 0; i < motors.length; i++) {
						motorSpeeds[i] = u * multipliers[i];
					}
					normalize(motorSpeeds);
					for (int i = 0; i < motors.length; i++) {
						motorSpeeds[i] *= 0.4;
					}
					for (int i = 0; i < motors.length; i++) {
						motors[i].set(motorSpeeds[i]);
					}
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					if (Math.abs(previousError - error) < 2) {
						t.start();
						System.out.println("testsstststss");
						if (t.hasPeriodPassed(0.1)) {
							System.out.println("test1151513531");
							autoMovement = false;
						}
					}
					if (Math.abs(error) < epsilon) {
						Timer.delay(0.25);
					}
					if (mode.equals(PIDModes.eRotate)) {
						error = rotateError(inputDevice.getYaw(), setPoint);
					} else if (mode.equals(PIDModes.eLinearX)) {
						error = linearError(setPoint, inputDevice.getDisplacementX() - initialDisplacement);
					} else {
						error = linearError(setPoint, inputDevice.getDisplacementY() - initialDisplacement);
					}
				}
			}

			System.out.println("exit: " + inputDevice.getYaw() + " | navX delat X: " + inputDevice.getDisplacementY());
			motors = null;
			inputDevice = null;
			autoMovement = false;
		}

		private double linearError(double point1, double point2) {
			return point2 - point1;
		}

		private double rotateError(double alpha, double beta) {
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