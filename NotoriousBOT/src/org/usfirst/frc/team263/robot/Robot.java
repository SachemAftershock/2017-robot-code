package org.usfirst.frc.team263.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends SampleRobot {

	AHRS gyro;
	Talon frontRight, backRight, ballShooterMotor;
	Spark frontLeft, backLeft;
	XboxController pDriver, sDriver;
	MecanumDrive drive;
	MechanismControls mech;
	BallShooter shooter;
	boolean fieldOriented, previouslyPressed;
	final double driftConstant;
	
	public Robot() {
		// Initialize motor controller addresses
		ballShooterMotor = new Talon(5);
		frontRight = new Talon(6);
		backRight = new Talon(7);
		frontLeft = new Spark(8);
		backLeft = new Spark(9);
		
		// Determine which motors are inverted empirically
		frontRight.setInverted(true);
		backRight.setInverted(false);
		frontLeft.setInverted(true);
		backLeft.setInverted(false);
		
		// Initialize navX MXP to be primary gyroscope
		gyro = new AHRS(SerialPort.Port.kMXP);
		
		// Initialize controllers to correct ports (!!!)
		pDriver = new XboxController(0);
		sDriver = new XboxController(1);
		
		// Initialize drift constant for rotational corrections in drivebase code
		driftConstant = 0.005;
		
		// Initialize all necessary systems and mechanisms
		drive = new MecanumDrive(frontRight, backRight, frontLeft, backLeft, gyro, driftConstant);
		shooter = new BallShooter(ballShooterMotor);
		mech = new MechanismControls(shooter);
		
		// Initialize booleans for field oriented toggle
		fieldOriented = false;
		previouslyPressed = false;
	}
	Timer t = new Timer();
	@Override
	public void operatorControl() {
		while(isOperatorControl() && isEnabled()) {
			// Determine if driver requests field-oriented driving or robot respective driving.
			if (!previouslyPressed && pDriver.getStickButton(Hand.kLeft)) {
				fieldOriented = !fieldOriented;
			}
			previouslyPressed = pDriver.getStickButton(Hand.kLeft);
			
			// Drive robot's drivebase and mechanisms.
			drive.drive(pDriver, fieldOriented);
			mech.drive(sDriver);
		}
	}
	
	@Override
	public void autonomous() {
		
	}
}
