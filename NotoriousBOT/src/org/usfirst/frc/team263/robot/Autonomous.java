package org.usfirst.frc.team263.robot;

public class Autonomous extends Thread {
	public enum autoStates {
		eDriveForward, eGear, eBackUp;
	}

	public Autonomous(MecanumDrive mc) {

	}

	public void run() {

	}

	public void rightGear(MecanumDrive drive, GearMechanism gearMechanism, BallShooter shooter) {
		/*while (climberSprocketLS.get()) {
			ropeClimber.pulse(0.3, 10);
		}
		ropeClimber.pulse(0, 0);
		// autonomousThread.start();
		drive.forward(0.3, 3300);
		try {
			Thread.sleep(1500);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		drive.autoRotate(-60);
		// drive.rotate(-0.4, 550);
		while (drive.autoMovement && isEnabled()) {

		}
		System.out.println("test");*/
		drive.forward(0.25, 1100);
		gearMechanism.toggleState();
		gearMechanism.run();
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			// e.printStackTrace();
		}
		drive.forward(-0.3, 2000);
		gearMechanism.toggleState();
		gearMechanism.run();
	}

	public void middleGearShoot(MecanumDrive drive, GearMechanism gearMechanism, BallShooter shooter) {
		drive.forward(0.25, 3900);
		gearMechanism.toggleState();
		gearMechanism.run();
		while (gearMechanism.getState().equals(GearMechanism.GearModes.eUp)) {
			// System.out.println("s");
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			// e.printStackTrace();
		}
		drive.forward(-0.3, 2200);
		gearMechanism.toggleState();
		gearMechanism.run();
		drive.strafe(-0.3, 500);
		shooter.setMotorPower();
		try {
			Thread.sleep(2200);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			// e.printStackTrace();
		}
		shooter.setAgitator(true);
		shooter.run();
	}
}
