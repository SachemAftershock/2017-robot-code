package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Autonomous {
	DigitalInput climberSprocketLS;
	RopeClimber climber;
	BallShooter shooter;
	GearMechanism gearMechanism;
	MecanumDrive drive;

	public Autonomous(MecanumDrive drive, GearMechanism gearMechanism, BallShooter shooter, RopeClimber climber, DigitalInput climberSprocketLS) {
		this.drive = drive;
		this.gearMechanism = gearMechanism;
		this.shooter = shooter;
		this.climber = climber;
		this.climberSprocketLS = climberSprocketLS;
	}

	public void leftGear() {
		climber.pulse(0.3, 2150);
		drive.forward(0.3, 3500);
		Timer.delay(1.5);
		drive.autoRotate(60);
		// drive.rotate(-0.4, 550);
		while (drive.autoMovement) {

		}
		drive.forward(0.25, 1400);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(1.5);
		drive.forward(-0.3, 2300);
		gearMechanism.toggleState();
		gearMechanism.run();
		drive.autoRotate(0);
		drive.forward(0.5, 1500);
	}

	public void rightGear() {
		climber.pulse(0.3, 2150);
		drive.forward(0.3, 3500);
		Timer.delay(1.5);
		drive.autoRotate(-60);
		// drive.rotate(-0.4, 550);
		while (drive.autoMovement) {

		}
		drive.forward(0.25, 1400);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(1.5);
		drive.forward(-0.3, 2300);
		gearMechanism.toggleState();
		gearMechanism.run();
		drive.autoRotate(0);
		drive.forward(0.5, 1500);
	}

	public void middleGearShoot() {
		climber.pulse(0.3, 2150);
		drive.forward(0.25, 4400);
		gearMechanism.toggleState();
		gearMechanism.run();
		long t = System.currentTimeMillis();
		while (!gearMechanism.getState().equals(GearMechanism.GearModes.eUp)) {
			if (System.currentTimeMillis() - t > 1500) break;
		}
		drive.forward(-0.3, 2900);
		gearMechanism.toggleState();
		gearMechanism.run();
		drive.strafe(-0.3, 700);
		shooter.setMotorPower(.97);
		Timer.delay(2.2);
		shooter.setAgitator(true);
		shooter.run();
	}
	
	public void middleGear() {
		climber.pulse(0.3, 2150);
		drive.forward(0.25, 4400);
		gearMechanism.toggleState();
		gearMechanism.run();
		long t = System.currentTimeMillis();
		while (!gearMechanism.getState().equals(GearMechanism.GearModes.eUp)) {
			if (System.currentTimeMillis() - t > 1500) break;
		}
		drive.forward(-0.3, 2900);
		gearMechanism.toggleState();
		gearMechanism.run();
	}
	
	public void leftGearStill() {
		climber.pulse(0.3, 2150);
		drive.forward(0.3, 3500);
		Timer.delay(1.5);
		drive.autoRotate(60);
		// drive.rotate(-0.4, 550);
		while (drive.autoMovement) {

		}
		drive.forward(0.25, 1400);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(0.5);
		drive.forward(-0.3, 2300);
		gearMechanism.toggleState();
		gearMechanism.run();
	}
	
	public void rightGearStill() {
		climber.pulse(0.3, 2150);
		drive.forward(0.3, 3500);
		Timer.delay(1.5);
		drive.autoRotate(-60);
		// drive.rotate(-0.4, 550);
		while (drive.autoMovement) {

		}
		drive.forward(0.25, 1400);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(0.5);
		drive.forward(-0.3, 2300);
		gearMechanism.toggleState();
		gearMechanism.run();
	}
}
