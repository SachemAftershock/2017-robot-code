package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * Class to retrieve data points from Raspberry Pi coprocessor using
 * NetworkTables
 * 
 * @version 1.0
 * @author Dan Waxman
 * @since 02-01-17
 */
public class CameraCoprocessor {
	private static NetworkTable table;
	private static boolean gearMode = true;

	/**
	 * @return (X,Y) of two contour points of gear
	 */
	public static double[] updateGearCamera() {
		table = NetworkTable.getTable("cameraData/gear");
		return new double[] { table.getNumber("pointOneX", -1.0), table.getNumber("pointOneY", -1.0),
				table.getNumber("pointTwoX", -1.0), table.getNumber("pointTwoY", -1.0) };
	}

	/**
	 * @return (X,Y) of two contour points of shooter
	 */
	public static double[] updateShooterCamera() {
		table = NetworkTable.getTable("cameraData/shooter");
		return new double[] { table.getNumber("pointOneX", -1.0), table.getNumber("pointOneY", -1.0),
				table.getNumber("pointTwoX", -1.0), table.getNumber("pointTwoY", -1.0) };
	}

	/**
	 * Toggles camera mode in NetworkTables for the Pi to send.
	 */
	public static void toggleClientCamera() {
		gearMode = !gearMode;
		table = NetworkTable.getTable("cameraData/clientMode");
		table.putBoolean("gearMode", gearMode);
	}
	
	/**
	 * Gets String for autonomous mode.
	 * @return String representing current autonomous mode.
	 */
	public static String getAutoMode() {
		table = NetworkTable.getTable("autoData");
		return table.getString("mode", "Middle With Shot");
	}
}