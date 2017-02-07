package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Master class for all mechanisms on Notorious B.O.T.
 * 
 * @author Dan Waxman
 * @version 0.1
 * @since 01-20-17
 */
public class MechanismControls {
	private BallIntake intake;
	private GearMechanism gearMechanism;
	private RopeClimber ropeClimber;
	private BallShooter shooter;
	private Macros macros;
	private boolean intakeAlreadyToggled, emergencyModeToggled, gearMechanismToggled, emergencyMode;

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
	public MechanismControls(BallShooter shooter, BallIntake intake, GearMechanism gearMechanism,
			RopeClimber ropeClimber, Macros macros) {
		this.shooter = shooter;
		this.intake = intake;
		this.gearMechanism = gearMechanism;
		this.ropeClimber = ropeClimber;
		this.macros = macros;
		intakeAlreadyToggled = false;
		emergencyModeToggled = false;
		gearMechanismToggled = false;
		emergencyMode = false;
	}

	/**
	 * Method to maintain and control all relevant subsystems.
	 * 
	 * @param controller
	 *            controller to read inputs from for various routines.
	 */
	public void drive(XboxController controller) {
		if (controller.getBumper(Hand.kLeft) && !intakeAlreadyToggled) {
			intake.toggleEnable();
		}
		if (controller.getBackButton() && !emergencyModeToggled) {
			emergencyMode = !emergencyMode;
		}
		if (emergencyMode) {
			LEDStrip.sendColor(LEDStrip.LEDMode.eRed);
			if (controller.getBButton()) {
				shooter.setMotorRPM(0.6);
			} else {
				shooter.setMotorRPM(0.0);
			}
			if (controller.getAButton() && gearMechanismToggled) {
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
			LEDStrip.sendColor(LEDStrip.LEDMode.eRainbow);
		}
		intakeAlreadyToggled = controller.getBumper(Hand.kLeft);
		emergencyModeToggled = controller.getBackButton();
		ropeClimber.updateEnable(controller.getXButton());

		intake.run();
		ropeClimber.run();
		gearMechanism.run();
	}
}
