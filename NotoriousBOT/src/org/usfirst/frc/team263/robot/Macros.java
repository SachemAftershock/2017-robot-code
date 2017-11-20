package org.usfirst.frc.team263.robot;

import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Class for handling any automated/macro actions during TeleOP
 * 
 * @version 0.1
 * @author Tyler Machado
 * @author Dan Waxman
 * @since 02-05-2017
 */
public class Macros {
	private AHRS gyro;
	private VisionProcessing vision;
	private MecanumDrive drive;
	private BallShooter shooter;
	private GearMechanism gearMechanism;
	private XboxController[] joysticks;
	private HashMap<Double, Double> shooterDistance;
	private final double MAX_FIRE_DISTANCE = 7.0; // to be found empirically?
	private int stateCounter;
	private boolean isRunning;
	private double distanceGearPeg, RPMNeeded;

	/**
	 * Instantiate Macros object
	 * 
	 * @param gyro
	 *            AHRS device to measure yaw
	 * @param cameraResX
	 *            Number of pixels in x-direction returned by camera
	 * @param cameraResY
	 *            Number of pixels in y-direction returned by camera
	 * @param drive
	 *            MecanumDrive object controlling robot's drivebase
	 * @param ballShooter
	 *            BallShooter object to control robot's flywheel
	 * @param gearMechanism
	 *            GearMechanism object to control Gear subsystem
	 * @param joysticks
	 *            XboxControllers to alert drivers on of any errors
	 */
	public Macros(AHRS gyro, int cameraResX, int cameraResY, MecanumDrive drive, BallShooter ballShooter,
			GearMechanism gearMechanism, XboxController[] joysticks) {
		this.gyro = gyro;
		this.shooter = ballShooter;
		this.gearMechanism = gearMechanism;
		this.drive = drive;
		this.joysticks = joysticks;
		shooterDistance = new HashMap<Double, Double>();
		vision = new VisionProcessing(cameraResX, cameraResY);
		stateCounter = 0;
		isRunning = false;
	}

	/**
	 * Starting from relatively close and facing the peg, can align the robot
	 * and deposit a loaded gear onto the peg
	 */
	public void gearPegMacro() {
		if (stateCounter == 0) {
			isRunning = true;
			stateCounter++;
		}
		if (isRunning) {
			if (stateCounter == 1) {
				drive.autoRotate(findClosestAngle(gyro.getYaw()));
				stateCounter++;
			}

			// Now it is assumed that we are facing the correct direction,
			// within
			// epsilon accuracy
			if (stateCounter == 2) {
				double[] centerPoints = CameraCoprocessor.updateGearCamera();
				if (Math.abs(centerPoints[0])  + Math.abs(centerPoints[1]) == 0) {
					for (XboxController js : joysticks) {
						JoystickRumble r = new JoystickRumble(js, 3);
						r.run();
					}
				} else {
					//strafeDist = vision.findStrafeDistancePeg(centerPoints);
					if (centerPoints[0] > 0.55) {
						drive.strafe(0.4, 300);
					} else if(centerPoints[1] < 0.45) {
						drive.strafe(-0.4, 300);
					} else { 
						stateCounter++;
					}
				}
			}
			if (stateCounter == 3) {
				drive.forward(0.5, 800);
				stateCounter++;
			}
			if (stateCounter == 4) {
				//drive.autoDrive(distanceGearPeg);
				stateCounter++;
			}
			if (stateCounter == 5) {
				gearMechanism.toggleState();
				stateCounter++;
			}
			if (stateCounter == 6) {
				Timer.delay(0.5);
				stateCounter++;
			}
			if (stateCounter == 7) {
				gearMechanism.toggleState();
				stateCounter = 0;
				isRunning = false;
			}
		}
	}

	/**
	 * Starting from facing the boiler, can shoot with the correct power.
	 */
	public void shooterMacro() {
		if (stateCounter == 0) {
			isRunning = true;
			stateCounter++;
		}
		if (stateCounter == 1) {
			double[] centerPoints = CameraCoprocessor.updateShooterCamera();
			double distance = vision.findDistanceBoiler(centerPoints);
			RPMNeeded = shooterDistance.get(distance);
			if (distance == -1 || distance > MAX_FIRE_DISTANCE) {
				(new JoystickRumble(joysticks, 2)).start();
				return;
			}
		}
		if (stateCounter == 2) {
			shooter.setMotorRPM(RPMNeeded);
			stateCounter++;
		}
		if (stateCounter == 3 && shooter.isUpToSpeed()) {
			shooter.setAgitator(true);
			stateCounter++;
		}
	}

	/**
	 * Disable everything currently running
	 */
	public void disableAll() {
		isRunning = false;
		stateCounter = 0;
		shooter.disable();
	}

	/**
	 * Given a current angle, find the closest peg angle
	 */
	private double findClosestAngle(double angle) {
		if (angle > 30) {
			return 60;
		}
		if (angle < -30) {
			return -60;
		}
		return 0;
	}
}