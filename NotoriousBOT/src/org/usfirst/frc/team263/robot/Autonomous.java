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
		while (climberSprocketLS.get()) {
			climber.pulse(0.3, 10);
		}
		drive.forward(0.3, 3250);
		Timer.delay(1.5);
		//drive.rotate(0.4, 550);
		drive.autoRotate(60);
	    while (drive.autoMovement) {
	    	
	    }
		drive.forward(0.25, 1100);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(0.5);
		drive.forward(-0.3, 2000);
		gearMechanism.toggleState();
		gearMechanism.run();
		drive.autoRotate(0);
		drive.forward(0.5, 1500);
	}

	public void rightGear() {
		while (climberSprocketLS.get()) {
			climber.pulse(0.3, 10);
		}
		drive.forward(0.3, 3300);
		Timer.delay(1.5);
		drive.autoRotate(-60);
		// drive.rotate(-0.4, 550);
		while (drive.autoMovement) {

		}
		drive.forward(0.25, 1100);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(0.5);
		drive.forward(-0.3, 2000);
		gearMechanism.toggleState();
		gearMechanism.run();
		drive.autoRotate(0);
		drive.forward(0.5, 1500);
	}

	public void middleGearShoot() {
		drive.forward(0.25, 3900);
		gearMechanism.toggleState();
		gearMechanism.run();
		while (!gearMechanism.getState().equals(GearMechanism.GearModes.eUp)) {
			continue;
		}
		drive.forward(-0.3, 2200);
		gearMechanism.toggleState();
		gearMechanism.run();
		drive.strafe(-0.3, 500);
		shooter.setMotorPower(0.97);
		Timer.delay(2.2);
		shooter.setAgitator(true);
		shooter.run();
	}
	
	public void middleGear() {
		drive.forward(0.25, 3900);
		gearMechanism.toggleState();
		gearMechanism.run();
		while (!gearMechanism.getState().equals(GearMechanism.GearModes.eUp)) {
			continue;
		}
		drive.forward(-0.3, 2200);
		gearMechanism.toggleState();
		gearMechanism.run();
		drive.strafe(-0.3, 500);
	}
	
	public void leftGearStill() {
		while (climberSprocketLS.get()) {
			climber.pulse(0.3, 10);
		}
		drive.forward(0.3, 3300);
		Timer.delay(1.5);
		drive.autoRotate(60);
		// drive.rotate(-0.4, 550);
		while (drive.autoMovement) {

		}
		drive.forward(0.25, 1100);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(0.5);
		drive.forward(-0.3, 2000);
		gearMechanism.toggleState();
		gearMechanism.run();
	}
	
	public void rightGearStill() {
		while (climberSprocketLS.get()) {
			climber.pulse(0.3, 10);
		}
		drive.forward(0.3, 3300);
		Timer.delay(1.5);
		drive.autoRotate(-60);
		// drive.rotate(-0.4, 550);
		while (drive.autoMovement) {

		}
		drive.forward(0.25, 1100);
		gearMechanism.toggleState();
		gearMechanism.run();
		Timer.delay(0.5);
		drive.forward(-0.3, 2000);
		gearMechanism.toggleState();
		gearMechanism.run();
	}
}
