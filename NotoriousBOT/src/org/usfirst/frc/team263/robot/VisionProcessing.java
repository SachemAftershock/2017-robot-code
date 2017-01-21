package org.usfirst.frc.team263.robot;

import java.util.ArrayList;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.vision.VisionRunner.Listener;

/**
 * Class for vision processing. Can be accessed to find the center of a target object.
 * @author Tyler Machado
 * @version b1.0
 */
public class VisionProcessing implements Listener<VelcroPipeline>
{
	private UsbCamera camera;
	private VelcroPipeline vp;
	private VisionThread visionThread;
	public CameraCalculations cc;
	
	public VisionProcessing()
	{
		camera = CameraServer.getInstance().startAutomaticCapture();
    	vp = new VelcroPipeline();
    	visionThread = new VisionThread(camera, vp, this);
    	visionThread.start();
    	cc = new CameraCalculations(160,120); //numbers hardcoded in for now, will probably be arguments in the end
	}
	
	/**
	 * Returns the point at the center of a given contour
	 * @param number The ID number of the contour being used. When you want to find mulitple center points, start from 0 and count up
	 */
	public Point takeCenterOfContour(int number)
	{
		Point ctr = null;
		ArrayList<MatOfPoint> cont = vp.filterContoursOutput();
		
		Point[] pts = cont.get(number).toArray();	
		Point BR = findBR(pts);
		Point TL = findTL(pts);
		ctr = cc.findCenterPoint(TL, BR);
				
		return ctr;	
	}
	
	private Point findBR(Point[] pts)
	{
		Point correct = pts[0];
		for(Point pt : pts)
		{
			if(pt.x >= correct.x && pt.y >= correct.y) correct = pt;
		}
		return correct;
	}
	
	private Point findTL(Point[] pts)
	{
		Point correct = pts[0];
		for(Point pt : pts)
		{
			if(pt.x <= correct.x && pt.y <= correct.y) correct = pt;
		}
		return correct;
	}
	
	@Override
	public void copyPipelineOutputs(VelcroPipeline pipeline) {
		// TODO Auto-generated method stub
		
	}
}
