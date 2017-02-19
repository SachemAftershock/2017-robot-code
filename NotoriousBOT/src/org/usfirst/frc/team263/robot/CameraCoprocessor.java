package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * Class to retrieve data points from Raspberry Pi coprocessor using NetworkTables
 * 
 * @version 1.0
 * @author Dan Waxman
 * @since 02-01-17
 */
public class CameraCoprocessor {
	private static NetworkTable table;
	private static boolean gearMode = true;
	
	public static double[] updateGearCamera() {
		table = NetworkTable.getTable("cameraData/gear");
		return new double[] {table.getNumber("pointOneX", -1.0), table.getNumber("pointOneY", -1.0), 
							table.getNumber("pointTwoX", -1.0), table.getNumber("pointTwoY", -1.0)};
	}
	
	public static double[] updateShooterCamera() {
		table = NetworkTable.getTable("cameraData/shooter");
		return new double[] {table.getNumber("pointOneX", -1.0), table.getNumber("pointOneY", -1.0), 
							table.getNumber("pointTwoX", -1.0), table.getNumber("pointTwoY", -1.0)};
	}
	
	public static void toggleClientCamera() {
		gearMode = !gearMode;
	}
	
	public static void setClientCamera() {
		table = NetworkTable.getTable("camera/clientMode");
		table.putBoolean("gearMode", gearMode);
	}
	
}