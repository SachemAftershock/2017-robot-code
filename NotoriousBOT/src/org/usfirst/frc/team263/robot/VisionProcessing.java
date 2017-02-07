package org.usfirst.frc.team263.robot;

import org.opencv.core.Point;

/**
 * Class for vision processing. Receives points representing the center of two
 * contours from the RPi and finds the distance from the corresponding target.
 * 
 * @author Tyler Machado
 * @version 1.1.0
 * @since 01-18-17
 */
public class VisionProcessing {
	public CameraCalculations cc;
	private int resX, resY;

	/**
	 * Instantiate VisionProcessing object
	 * 
	 * @param resX
	 *            Resolution of camera in x direction
	 * @param resY
	 *            Resolution of camera in y direction
	 */
	public VisionProcessing(int resX, int resY) {
		cc = new CameraCalculations(resX, resY);
		this.resX = resX;
		this.resY = resY;
	}

	/**
	 * Use for two tape pieces horizontally apart, specifically the peg
	 * 
	 * @param pt1
	 *            Left justified contour
	 * @param pt2
	 *            Right justified contour
	 * @return Ratio between the two peg contours
	 */
	public double findPixelsPerInchPeg(Point pt1, Point pt2) {
		double deltaX = Math.abs(pt2.x - pt1.x);
		double ratio = deltaX / 8.25;
		return ratio;
	}

	/**
	 * Use for two tape pieces vertically apart, specifically the boiler
	 * 
	 * @param pt1
	 *            Lower justified contour
	 * @param pt2
	 *            Upper justified contour
	 * @return Ratio between the two boiler contours
	 */
	public double findPixelsPerInchBoiler(Point pt1, Point pt2) {
		double deltaY = Math.abs(pt2.y - pt1.y);
		double ratio = deltaY / 7.0;
		return ratio;
	}

	/**
	 * Find the ground-bound distance between the camera and the gear-peg
	 * 
	 * @param nums
	 *            X and Y values of two contours
	 * @return Distance from peg (inches)
	 */
	public double findDistancePeg(double[] nums) {
		if (nums[3] < 0) {
			return -1.0; // error handing from the Pi
		}

		Point[] pts = arrToPoints(nums);
		double PPI = findPixelsPerInchPeg(pts[0], pts[1]);
		double width = resX / PPI;
		double distDiag = .5 * width / Math.tan(25 * Math.PI / 180); // degrees
																		// to
																		// radians
		double distLat = Math.sqrt(distDiag * distDiag - 189.0625); // use
																	// triangulation
																	// to find
																	// the
																	// lateral
																	// distance,
																	// magic
																	// number is
																	// deltaH
																	// squared
		return distLat;
	}

	/**
	 * Find the ground-bound distance between the camera and the boiler
	 * 
	 * @param nums
	 *            X and Y values of contours
	 * @return Distance from boiler (inches)
	 */
	public double findDistanceBoiler(double[] nums) {
		if (nums[3] < 0)
			return -1.0; // error handing from the Pi

		Point[] pts = arrToPoints(nums);
		double PPI = findPixelsPerInchBoiler(pts[0], pts[1]);
		double width = resY / PPI;
		double distDiag = .5 * width / Math.tan(19.0935 * Math.PI / 180); // degrees
																			// to
																			// radians
		double distLat = Math.sqrt(distDiag * distDiag - 3422.25); // use
																	// triangulation
																	// to find
																	// the
																	// lateral
																	// distance,
																	// magic
																	// number is
																	// deltaH
																	// squared
		return distLat;
	}

	/**
	 * Given the distance away from the peg and letting rotation be
	 * perpendicular to the peg, can calculate the distance needed to strafe in
	 * order to center the peg in the camera
	 * 
	 * @param nums
	 *            X and Y values of points
	 * @return Distance to strafe for peg (inches)
	 */
	public double findStrafeDistancePeg(double[] nums) {
		Point[] pts = arrToPoints(nums);
		double PPI = findPixelsPerInchPeg(pts[0], pts[1]);
		Point ctr = cc.findCenterPoint(pts[0], pts[1]);
		double pixels = ctr.x - resX / 2;
		return pixels / PPI;
	}

	/**
	 * The RPi will return data in the format of four doubles; this converts
	 * that to two points
	 * 
	 * @param nums
	 *            X and Y values of points
	 * @return Two Point objects reflecting nums
	 */
	private Point[] arrToPoints(double[] nums) {
		Point pt1 = new Point(nums[0], nums[1]);
		Point pt2 = new Point(nums[2], nums[3]);
		return new Point[] { pt1, pt2 };
	}
}
