package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Autonomous {
	DigitalInput climberSprocketLS;
	RopeClimber climber;
	BallShooter shooter;
	GearMechanism gearMechanism;
	MecanumDrive drive;

	public Autonomous(MecanumDrive drive, GearMechanism gearMechanism, BallShooter shooter, RopeClimber climber,
			DigitalInput climberSprocketLS) {
		this.drive = drive;
		this.gearMechanism = gearMechanism;
		this.shooter = shooter;
		this.climber = climber;
		this.climberSprocketLS = climberSprocketLS;
	}

	public void leftGear() {
		(new Thread() {
			public void run() {
				climber.pulse(0.3, 2150);
			}
		}).start();
		drive.forward(0.3, 3850);
		Timer.delay(1.5);
		drive.autoRotate(60);
		// drive.rotate(-0.4, 550);
		long t = System.currentTimeMillis();
		while (drive.autoMovement) {
			if (System.currentTimeMillis() - t > 1500) {
				drive.autoMovement = false;
				break;
			}
		}
		drive.forward(0.25, 2000);
		drive.forward(-0.25, 200);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(1.5);
		drive.forward(-0.3, 2300);
		gearMechanism.toggleState();
		gearMechanism.run();
		drive.autoRotate(0);
		t = System.currentTimeMillis();
		while (drive.autoMovement) {
			if (System.currentTimeMillis() - t > 1500) {
				drive.autoMovement = false;
				break;
			}
		}
		drive.forward(0.6, 2700);
	}

	public void rightGear() {
		(new Thread() {
			public void run() {
				climber.pulse(0.3, 2150);
			}
		}).start();
		drive.forward(0.23, 4050);
		Timer.delay(1.5);
		drive.autoRotate(-60);
		// drive.rotate(-0.4, 550);
		long t = System.currentTimeMillis();
		while (drive.autoMovement) {
			if (System.currentTimeMillis() - t > 1900) {
				drive.autoMovement = false;
				break;
			}
		}
		drive.forward(0.25, /*2000*/2500);
		drive.forward(-0.25, 2500);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(1.5);
		drive.forward(-0.3, 2300);
		gearMechanism.toggleState();
		gearMechanism.run();
		drive.autoRotate(0);
		t = System.currentTimeMillis();
		while (drive.autoMovement) {
			if (System.currentTimeMillis() - t > 1500) {
				drive.autoMovement = false;
				break;
			}
		}
		drive.forward(0.6, 2700);
	}

	public void middleGearShoot() {
		(new Thread() {
			public void run() {
				climber.pulse(0.3, 2150);
			}
		}).start();
		//drive.forward(0.25, 4400);
		drive.forward(0.2, /*5400*/5700);
		drive.forward(-0.2, 200);
		gearMechanism.toggleState();
		gearMechanism.run();
		long t = System.currentTimeMillis();
		while (!gearMechanism.getState().equals(GearMechanism.GearModes.eUp)) {
			if (System.currentTimeMillis() - t > 1500)
				break;
		}
		drive.forward(-0.3, 2900);
		gearMechanism.toggleState();
		gearMechanism.run();
		drive.strafe(-0.3, 700);
		shooter.setMotorPower(.98);
		Timer.delay(2.2);
		shooter.setAgitator(true);
		shooter.run();
	}

	public void middleGear() {
		(new Thread() {
			public void run() {
				climber.pulse(0.3, 2150);
			}
		}).start();
		drive.forward(0.2, /*5400*/6800);
		drive.forward(-0.2, 200);
		gearMechanism.toggleState();
		gearMechanism.run();
		long t = System.currentTimeMillis();
		while (!gearMechanism.getState().equals(GearMechanism.GearModes.eUp)) {
			if (System.currentTimeMillis() - t > 1500)
				break;
		}
		drive.forward(-0.3, 2900);
		gearMechanism.toggleState();
		gearMechanism.run();
	}

	public void leftGearStill() {
		(new Thread() {
			public void run() {
				climber.pulse(0.3, 2150);
			}
		}).start();
		drive.forward(0.3, 3500);
		Timer.delay(1.5);
		drive.autoRotate(60);
		// drive.rotate(-0.4, 550);
		long t = System.currentTimeMillis();
		while (drive.autoMovement) {
			if (System.currentTimeMillis() - t > 1500) {
				break;
			}
		}
		drive.forward(0.25, /*2000*/2500);
		drive.forward(-0.25, 100);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(0.5);
		drive.forward(-0.3, 2300);
		gearMechanism.toggleState();
		gearMechanism.run();
	}

	public void rightGearStill() {
		(new Thread() {
			public void run() {
				climber.pulse(0.3, 2150);
			}
		}).start();
		drive.forward(0.3, 3500);
		Timer.delay(1.5);
		drive.autoRotate(-60);
		// drive.rotate(-0.4, 550);
		long t = System.currentTimeMillis();
		while (drive.autoMovement) {
			if (System.currentTimeMillis() - t > 1500) {
				break;
			}
		}
		drive.forward(0.25, /*2000*/2500);
		drive.forward(-.25, 200);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(0.5);
		drive.forward(-0.3, 2300);
		gearMechanism.toggleState();
		gearMechanism.run();
	}
}
