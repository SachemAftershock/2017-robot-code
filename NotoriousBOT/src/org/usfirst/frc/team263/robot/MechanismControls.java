package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Master class for all mechanisms on Notorious B.O.T.
 * @author Dan Waxman
 * @version 0.1
 * @since 01-20-17
 */
public class MechanismControls {
	private BallIntake intake;
	private GearMechanism gearMechanism;
	private RopeClimber ropeClimber;
	private Macros macros;
	private boolean intakeAlreadyToggled;
	
	//TODO create more efficient controls and semi-autonomous routines to support them.
	
	/**
	 * Instantiates master class.
	 * @param shooter -- BallShooter object to control.
	 */
	public MechanismControls(BallIntake intake, GearMechanism gearMechanism, RopeClimber ropeClimber, Macros macros) {
		this.intake = intake;
		this.gearMechanism = gearMechanism;
		this.ropeClimber = ropeClimber;
		this.macros = macros;
		intakeAlreadyToggled = false;
	}
	
	/**
	 * Method to maintain and control all relevant subsystems.
	 * @param controller -- controller to read inputs from for various routines.
	 */
	public void drive(XboxController controller) {
		ropeClimber.updateEnable(controller.getXButton());
		if (controller.getBumper(Hand.kLeft) && !intakeAlreadyToggled) {
			intake.toggleEnable();
		}
		if (controller.getBButton()) {
			macros.gearPegMacro();
		} else if (controller.getAButton()) {
			macros.shooterMacro();
		} else {
			macros.disableAll();
		}
		intakeAlreadyToggled = controller.getBumper(Hand.kLeft);
		
		
		intake.run();
		ropeClimber.run();
		gearMechanism.run();
	}
}
