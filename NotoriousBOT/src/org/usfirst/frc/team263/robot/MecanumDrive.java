package org.usfirst.frc.team263.robot;

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
	private AHRS mGyro;
	private final double TUNED_KP = 0.0071, TUNED_KI = 0.004, TUNED_KD = -0.0025, TUNED_KF = 0.02,
			ROTATION_CONSTANT = 0.6;

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

			// "Drive straight" mode to avoid slower movement due to strafing
			if (controller.getBumper(Hand.kRight)) {
				x = 0;
			}

			// Drift correction using proportional gain
			if (r == 0 && Math.abs(x) + Math.abs(y) > 0) {
				r = kd * mGyro.getRate();
			}

			if (fieldCentric) {
				// Perform vector rotation in R^2 by theta degrees
				// This can be useful for things such as aligning a rope climb
				// Where relative movement can be confusing
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

			// Throttle for finer movement
			double throttleMultiplier = controller.getBumper(Hand.kLeft) ? 0.5 : 1.0;

			// Set finalized speeds
			mFrontRight.set(speeds[0] * throttleMultiplier);
			mBackRight.set(speeds[1] * throttleMultiplier);
			mFrontLeft.set(speeds[2] * throttleMultiplier);
			mBackLeft.set(speeds[3] * throttleMultiplier);

			// Automovement threads
			// Automovement is a public variable used in place of thread
			// interrupts. These line up perfectly for peg placement.
			if (controller.getXButton()) {
				synchronized (this) {
					autoMovement = true;
					rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, TUNED_KF, mGyro, 60,
							new SpeedController[] { mFrontRight, mBackRight, mFrontLeft, mBackLeft },
							new double[] { 1, 1, -1, -1 });
					rotationThread.start();
				}
			} else if (controller.getBButton()) {
				synchronized (this) {
					autoMovement = true;
					rotationThread = new PIDController(TUNED_KP, TUNED_KI, TUNED_KD, TUNED_KF, mGyro, -60,
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
			// Cancels and effectively kills any running rotation threads.
			autoMovement = false;
		}
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

	/**
	 * Method to move linearly for a certain period of time.
	 * 
	 * @param speed
	 *            Speed to set motors at to move -- positive is forward,
	 *            negative is backward
	 * @param time
	 *            Time period to move for (in ms).
	 */
	public void forward(double speed, int time) {
		mFrontLeft.set(speed);
		mFrontRight.set(speed);
		mBackLeft.set(speed);
		mBackRight.set(speed);
		Timer.delay((double)time / 1000.0);
		mFrontLeft.set(0);
		mFrontRight.set(0);
		mBackLeft.set(0);
		mBackRight.set(0);
	}

	/**
	 * Method to strafe for a certain period of time.
	 * 
	 * @param speed
	 *            Speed to set motors at to strafe -- positive is to the right,
	 *            negative to the left.
	 * @param time
	 *            Time period to move for (in ms).
	 */
	public void strafe(double speed, long time) {
		mFrontRight.set(-speed);
		mFrontLeft.set(speed);
		mBackRight.set(speed);
		mBackLeft.set(-speed);
		Timer.delay(time / 1000);
		mFrontRight.set(0);
		mFrontLeft.set(0);
		mBackRight.set(0);
		mBackLeft.set(0);
	}

	/**
	 * Method to rotate for a certain period of time.
	 * 
	 * @param speed
	 *            Speed to set motors at to rotate -- positive is clockwise,
	 *            negative to the counterclockwise.
	 * @param time
	 *            Time period to move for (in ms).
	 */
	public void rotate(double speed, long time) {
		mFrontRight.set(-speed);
		mFrontLeft.set(speed);
		mBackRight.set(-speed);
		mBackLeft.set(speed);
		Timer.delay(time / 1000);
		mFrontRight.set(0);
		mFrontLeft.set(0);
		mBackRight.set(0);
		mBackLeft.set(0);
	}

	/**
	 * Method to create a PIDF thread for rotation.
	 * 
	 * @param theta
	 *            Absolute position to field to rotate to (in degrees)
	 */
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
	 * PIDF Class for auto base rotation
	 * 
	 * Feedforward is provided for angular acceleration
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

		/**
		 * Constructor for new PIDF thread
		 * 
		 * @param Kp
		 *            Tuned proportional gain constant
		 * @param Ki
		 *            Tuned integral gain constant
		 * @param Kd
		 *            Tuned derivative gain constant
		 * @param Kf
		 *            Tuned feedforward gain constant
		 * @param gyro
		 *            AHRS device for feedback
		 * @param setPoint
		 *            Angle (in degrees) to rotate to
		 * @param motors
		 *            Set of SpeedController devices for output
		 * @param multipliers
		 *            Multipliers to apply to each motor
		 */
		public PIDController(double Kp, double Ki, double Kd, double Kf, AHRS gyro, double setPoint,
				SpeedController[] motors, double[] multipliers) {
			this.kp = Kp;
			this.ki = Ki;
			this.kd = Kd;
			this.kf = Kf;
			this.setPoint = setPoint;
			this.motors = motors;
			this.multipliers = multipliers;
			this.inputDevice = gyro;
		}

		/**
		 * Method for closed loop rotation
		 */
		public void run() {
			error = rotationalError(inputDevice.getYaw(), setPoint);
			previousError = error;
			alpha = 0;
			previousOmega = inputDevice.getRate();
			while (Math.abs(error) > epsilon && autoMovement) {
				synchronized (this) {
					integral += error;
					alpha = inputDevice.getRate() - previousOmega;
					previousOmega = inputDevice.getRate();
					double iTerm = Math.abs(previousError - error) < 2 ? ki * integral : 0;
					double u = kp * error + iTerm + kd * (previousError - error) - kf * Math.abs(alpha);
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
			// Release objects taking significant memory
			motors = null;
			inputDevice = null;
			autoMovement = false;
		}

		/**
		 * Method to find signed error in rotation in 2D plane
		 * 
		 * @param alpha
		 *            First angle
		 * @param beta
		 *            Second angle
		 * @return Absolute signed error in 2D plane
		 */
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
}