package org.usfirst.frc.team263.robot;

import java.net.ServerSocket;
import java.net.Socket;

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
	//private static ServerSocket server;
	//private static Socket client;

	public static double[] updateGearCamera() {
		table = NetworkTable.getTable("cameraData/gear");
		return new double[] {table.getNumber("pointOneX", -1.0),
		table.getNumber("pointOneY", -1.0),
		table.getNumber("pointTwoX", -1.0), table.getNumber("pointTwoY", -1.0)};
	}

	public static double[] updateShooterCamera() {
		table = NetworkTable.getTable("cameraData/shooter");
		return new double[] {table.getNumber("pointOneX", -1.0),
		table.getNumber("pointOneY", -1.0),
		table.getNumber("pointTwoX", -1.0), table.getNumber("pointTwoY", -1.0)};
	}
	/*
	public static void connectToggle() {
		try {
			server = new ServerSocket(5800);
			// client = server.accept();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}*/

	public static void toggleClientCamera() {
		gearMode = !gearMode;
		table = NetworkTable.getTable("cameraData/clientMode");
		table.putBoolean("gearMode", gearMode);
	}

	/*public static void setClientCamera() {
		try {
			if (gearMode) {
				client.getOutputStream().write(236);
				System.out.println("toggling camera");
			}
		} catch (Exception e) {
			e.printStackTrace();
		}

	}
*/
	private class Server extends Thread {
		public Server() {

		}

		public void run() {

		}
	}
}