package org.usfirst.frc.team263.robot;
import org.opencv.core.Point;

/**
 * Class for calculations regarding a found point. Can convert to a cartesian or polar point and return various values
 * @author Tyler Machado
 * @version 1.0
 */
public class CameraCalculations 
{
	private double resX, resY;
	
	public CameraCalculations(double resX, double resY)
	{
		this.resX = resX;
		this.resY = resY;
	}
	
	/**
	 * Given a point of the camera's native co-ords, returns a modified one with a central origin and a domain/range of [-1,1]
	 */
	public Point rawToScaled(Point orig)
	{
		double rawX = orig.x;
		double rawY = orig.y;
		double x = rawX/(resX*.5)-1;
		double y = -1*(rawY/(resY*.5)-1);
		
		Point pt = new Point(x,y);
		return pt;
	}

	/**
	 * Given a converted point, finds the distance to move that (when combined with the findAngleDegrees) will center the target at (0,0)
	 */
	public double findDistanceToCenter(Point orig)
	{
		double x = orig.x;
		double y = orig.y;
		
		double dist = Math.sqrt(Math.pow(x,2) + Math.pow(y, 2));
		
		return dist;
	}
	
	/**
	 * Given a converted point, finds the angle to move that (when combined with the findDistanceToCenter) will center the target at (0,0)
	 */
	public double findAngleDegrees(Point orig)
	{
		double x = orig.x;
		double y = orig.y;
		
		double angle = Math.atan2(y, x)*180/Math.PI; //atan gives radians, convert to degrees
		
		if(angle < 0) angle+=360; //make all values positive in the range [0,360]
		
		return angle;
	}
	
	/**
	 * Given a converted point, returns the polar (r, theta) of its location relative to the center
	 */
	public Point findPolarPoint(Point orig)
	{
		double r = findDistanceToCenter(orig);
		double theta = findAngleDegrees(orig);
		
		return new Point(r, theta);
	}
	
	/**
	 * Given two points, finds the exact center between them
	 */
	public Point findCenterPoint(Point p1, Point p2)
	{
		double x = (p1.x + p2.x)/2;
		double y = (p1.y + p2.y)/2;
		
		return new Point(x, y);
	}
	
	public static void main(String[] args) //testing
	{
		CameraCalculations CC = new CameraCalculations(320, 240);
		
		Point pt = new Point(270,140);
		pt = CC.rawToScaled(pt);
		Point pt2 = CC.findPolarPoint(pt);
		System.out.println(pt2);
	}	
}
