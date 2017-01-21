package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Master class for all mechanisms on Notorious B.O.T.
 * @author Dan Waxman
 * @version 0.1
 * @since 01-20-17
 */
public class MechanismControls {
	BallShooter shooter;
	
	//TODO create more efficient controls and semi-autonomous routines to support them.
	
	/**
	 * Instantiates master class.
	 * @param shooter -- BallShooter object to control.
	 */
	public MechanismControls(BallShooter shooter) {
		this.shooter = shooter;
	}
	
	/**
	 * Method to maintain and control all relevant subsystems.
	 * @param controller -- controller to read inputs from for various routines.
	 */
	public void drive(XboxController controller) {
		shooter.setMotorSpeed(controller.getRawAxis(1));
	}
}
