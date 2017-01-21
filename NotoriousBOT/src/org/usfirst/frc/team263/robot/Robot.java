package org.usfirst.frc.team263.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends SampleRobot {

	AHRS gyro;
	Talon frontRight, backRight, ballShooterMotor;
	Spark frontLeft, backLeft;
	XboxController pDriver, sDriver;
	MecanumDrive drive;
	MechanismControls mech;
	BallShooter shooter;

	public Robot() {
		ballShooterMotor = new Talon(5);
		frontRight = new Talon(6);
		backRight = new Talon(7);
		frontLeft = new Spark(8);
		backLeft = new Spark(9);
		
		frontRight.setInverted(false);
		backRight.setInverted(false);
		frontLeft.setInverted(false);
		backLeft.setInverted(false);
		
		gyro = new AHRS(SerialPort.Port.kMXP);
		
		pDriver = new XboxController(0);
		sDriver = new XboxController(1);
		
		drive = new MecanumDrive(frontRight, backRight, frontLeft, backLeft);
		shooter = new BallShooter(ballShooterMotor);
		mech = new MechanismControls(shooter);
	}
	
	@Override
	public void operatorControl() {
		while(isOperatorControl() && isEnabled()) {
			drive.drive(pDriver, gyro);
			mech.drive(sDriver);
		}
	}
	
	@Override
	public void autonomous() {
		
	}
}
