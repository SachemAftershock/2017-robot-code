package org.usfirst.frc.team263.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Drivebase code for a mecanum drivebase. Supports field-centric and non-oriented driving methods.
 * 
 * @version 1.0
 * @author Dan Waxman
 * @since 01-20-17
 */
public class MecanumDrive {
	private SpeedController mFrontRight, mBackRight, mFrontLeft, mBackLeft;
	
	/**
	 * Construct instance of drivebase code with appropriate motor controllers to be addressed.
	 * 
	 * @param frontRight -- front right wheel motor controller
	 * @param backRight -- back right wheel motor controller
	 * @param frontLeft -- front left wheel motor controller
	 * @param backLeft -- back left wheel motor controller
	 */
	public MecanumDrive(SpeedController frontRight, SpeedController backRight, SpeedController frontLeft, SpeedController backLeft) {
		mFrontRight = frontRight;
		mBackRight = backRight;
		mFrontLeft = frontLeft;
		mBackLeft = backLeft;
	}
	
	/**
	 * Method to drive a mecanum drivebase off of Xbox Controller input. 
	 * 
	 * <p> This method is non-field oriented, but normalizes inputs to create an efficient movement system.</p>
	 * <p> Refer to <code>drive(Xbox controller, AHRS gyro)</code> for field oriented controls.</p>
	 * 
	 * @param controller -- Xbox controller to input drive controls 
	 */
	public void drive(XboxController controller) {
		// Get controller inputs with artifical deadbands.
		// y-axis is inverted in order to make driving more intuitive.
		double x = deadband(controller.getRawAxis(0), 0.1);
		double y = deadband(-controller.getRawAxis(1), 0.1);
		double r = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
		
		// Speeds = {fr, br, fl, bl} operations for each wheel speed. 
		// Speeds are then normalized to make sure the robot drives correctly.
		double[] speeds = {-x + y - r, x + y - r, x + y + r, -x + y + r};
		normalize(speeds);
		
		mFrontRight.set(speeds[0]);
		mBackRight.set(speeds[1]);
		mFrontLeft.set(speeds[2]);
		mBackLeft.set(speeds[3]);
	}
	
	/**
	 * Method to drive a mecanum drivebase off of Xbox Controller input with field-centric controls. 
	 * 
	 * <p> This method is field oriented, meaning that a control direction is absolute to the field's Cartesian plane.</p>
	 * <p> Refer to <code>drive(Xbox controller)</code> for non-field oriented controls.</p>
	 * 
	 * @param controller -- Xbox controller to input drive controls 
	 * @param gyro -- NavX device to read current angle in relation to the field from.
	 */
	public void drive(XboxController controller, AHRS gyro) {
		// Get controller inputs with artificial deadband. 
		// y-axis is negated in order to make driving more intuitive.
		double x = deadband(controller.getRawAxis(0), 0.1);
		double y = deadband(-controller.getRawAxis(1), 0.1);
		double r = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
		
		// Perform vector rotation in R^2 by theta degrees
		double theta = gyro.getYaw();
		double sinT = Math.sin(Math.toRadians(theta));
		double cosT = Math.cos(Math.toRadians(theta));
		double yPrime = x * sinT + y * cosT;
		x = x * cosT - y * sinT;
		y = yPrime;
		
		// Speeds = {fr, br, fl, bl} operations for each wheel speed. 
		// Speeds are then normalized to make sure the robot drives correctly.
		double[] speeds = {-x + y - r, x + y - r, x + y + r, -x + y + r};
		normalize(speeds);
		
		mFrontRight.set(speeds[0]);
		mBackRight.set(speeds[1]);
		mFrontLeft.set(speeds[2]);
		mBackLeft.set(speeds[3]);
	}
	
	/**
	 * Normalizes an array of values to [-1,1] scale.
	 * 
	 * <p>Optimal for motor normalization to create ranged speed control values</p>
	 * 
	 * @param array -- array of values to normalize from [-1,1] scale
	 */
	private void normalize(double[] array) {
		boolean normFlag = false;
		double maxValue = array[0];
		
		for(double value : array) {
			if (Math.abs(value) > maxValue) {
				maxValue = Math.abs(value);
				normFlag = maxValue > 1;
			}
		}
		
		if (normFlag) {
			for(int i = 0; i < array.length; i++) {
				array[i] /= maxValue;
			}
		}
	}
	
	/**
	 * Creates artificial absolute deadband on values. 
	 * 
	 * @param value -- value to create deadband on
	 * @param deadband -- minimum number that <code>abs(value)</code> must exceed
	 * @return value if <code>abs(value)</code> is greater than deadband, 0 otherwise
	 */
	private double deadband(double value, double deadband) {
		return Math.abs(value) > deadband ? value : 0.0;
	}
}
