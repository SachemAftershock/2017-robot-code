package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Master class for all mechanisms on Notorious B.O.T.
 * 
 * @author Dan Waxman
 * @version 0.1
 * @since 01-20-17
 */
public class MechanismControls {
	private GearMechanism gearMechanism;
	private RopeClimber ropeClimber;
	private BallShooter shooter;
	private Macros macros;
	private Servo servo;
	private boolean emergencyModeToggled, gearMechanismToggled, emergencyMode, clientCameraToggled;
	//private double servoSP;

	/**
	 * Instantiates master class
	 * 
	 * @param shooter
	 *            BallShooter subsystem object
	 * @param intake
	 *            BallIntake mechanism object
	 * @param gearMechanism
	 *            GearMechanism subsystem object
	 * @param ropeClimber
	 *            RopeClimber subsystem object
	 * @param macros
	 *            Macros container for all semi-autonomous routines
	 */
	public MechanismControls(BallShooter shooter, GearMechanism gearMechanism,
			RopeClimber ropeClimber, Macros macros, Servo servo) {
		this.shooter = shooter;
		this.gearMechanism = gearMechanism;
		this.ropeClimber = ropeClimber;
		this.macros = macros;
		this.servo = servo;
		emergencyModeToggled = false;
		gearMechanismToggled = false;
		emergencyMode = true;
		clientCameraToggled = false;
		//servoSP = 0.70;
	}

	/**
	 * Method to maintain and control all relevant subsystems.
	 * 
	 * @param controller
	 *            controller to read inputs from for various routines.
	 */
	public void drive(XboxController controller) {
		if (controller.getBackButton() && !emergencyModeToggled) {
			emergencyMode = !emergencyMode;
		}
		if (controller.getBumper(Hand.kLeft)) {
			ropeClimber.pulse(0.3, 100);
		}
		if (/*emergencymode*/true) {
			LEDStrip.sendColor(LEDStrip.LEDMode.eRed);
			if (controller.getBButton()) {
				shooter.setMotorPower(0.70);
				shooter.setAgitator(controller.getBumper(Hand.kRight));
			} else {
				shooter.setMotorPower(0.0);
				shooter.setAgitator(false);
			}
			if (controller.getAButton() && !gearMechanismToggled) {
				gearMechanism.toggleState();
			}
			if(controller.getPOV() == 180) {
				servo.set(1.0);
				
			} else if(controller.getPOV() == 0) {
				servo.set(0.7);
			}
			if(controller.getStartButton() && clientCameraToggled) {
				CameraCoprocessor.toggleClientCamera();
			}
			
		} else {
			if (controller.getBButton()) {
				macros.gearPegMacro();
			} else if (controller.getAButton()) {
				macros.shooterMacro();
			} else {
				macros.disableAll();
			}
			LEDStrip.sendColor(LEDStrip.LEDMode.eRainbow);
		}
		emergencyModeToggled = controller.getBackButton();
		ropeClimber.updateEnable(controller.getXButton());
		gearMechanismToggled = controller.getAButton();
		clientCameraToggled = controller.getStartButton();
		
		

		ropeClimber.run();
		gearMechanism.run();
		shooter.run();
		CameraCoprocessor.setClientCamera();
	}
}