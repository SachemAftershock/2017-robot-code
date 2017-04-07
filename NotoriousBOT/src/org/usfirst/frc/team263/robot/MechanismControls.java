package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.VictorSP;
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
	private VictorSP hopperMotor;
	private boolean emergencyModeToggled, gearMechanismToggled, emergencyMode, clientCameraToggled;
	private double pos;
	private int counter;

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
	public MechanismControls(BallShooter shooter, GearMechanism gearMechanism, RopeClimber ropeClimber, Macros macros,
			Servo servo, VictorSP hopperMotor) {
		this.shooter = shooter;
		this.gearMechanism = gearMechanism;
		this.ropeClimber = ropeClimber;
		this.macros = macros;
		this.servo = servo;
		this.hopperMotor = hopperMotor;
		emergencyModeToggled = false;
		gearMechanismToggled = false;
		emergencyMode = true;
		clientCameraToggled = false;
		pos = 0.0;
		counter = 0;
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
			ropeClimber.setMaxSpeed(0.4);
		} else {
			ropeClimber.setMaxSpeed(1.0);
		}
		if (true/*emergencyMode*/) {
			if (controller.getBButton()) {
				shooter.setMotorPower();
				shooter.setAgitator(controller.getBumper(Hand.kRight));
				hopperMotor.set(controller.getTriggerAxis(Hand.kRight));
			} else {
				shooter.setMotorRPM(0);
				shooter.setAgitator(false);
				hopperMotor.set(0);
				shooter.disable();
			}
			if (controller.getAButton() && !gearMechanismToggled) {
				gearMechanism.toggleState();
			}
		} else {
			if (controller.getBButton()) {
				macros.gearPegMacro();
			} else if (controller.getAButton()) {
				macros.shooterMacro();
			} else {
				macros.disableAll();
			}
		}
		
		if(controller.getBackButton())
			counter++;
		servo.set(counter % 2 == 0 ? 0.85 : 0.4);
		
			
		
		//.85 = down
		//.65 or .4 = up? idr
		
		
		
		if(controller.getStartButton() && !clientCameraToggled) {
			CameraCoprocessor.toggleClientCamera();
		}
		
		emergencyModeToggled = controller.getBackButton();
		ropeClimber.updateEnable(controller.getXButton());
		gearMechanismToggled = controller.getAButton();
		clientCameraToggled = controller.getStartButton();

		ropeClimber.run();
		gearMechanism.run();
		shooter.run();
	}
}