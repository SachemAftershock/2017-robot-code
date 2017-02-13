package org.usfirst.frc.team263.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends SampleRobot {

	AHRS gyro;
	VictorSP frontRight, frontLeft, backRight, backLeft, agitator; 
	CANTalon ballShooterMotor, ropeClimberMotor;
	VictorSP gearMechanismMotor;
	XboxController pDriver, sDriver;
	MecanumDrive drive;
	MechanismControls mech;
	BallShooter shooter;
	RopeClimber ropeClimber;
	GearMechanism gearMechanism;
	DigitalInput leftClimberLS, rightClimberLS, climberSprockectLS, bottomGearLS, topGearLS;
	Encoder shooterEncoder;
	Macros macros;
	boolean fieldOriented, previouslyPressed;
	final double DRIFT_CONSTANT = 0.005;
	final int CAMERA_X = 640, CAMERA_Y = 480;

	public Robot() {
		// Initialize motor controller addresses
		ballShooterMotor = new CANTalon(0);
		ropeClimberMotor = new CANTalon(2);
		frontRight = new VictorSP(0);
		backRight = new VictorSP(1);
		frontLeft = new VictorSP(2);
		backLeft = new VictorSP(3);
		gearMechanismMotor = new VictorSP(4);
		agitator = new VictorSP(5);

		// Initialize all DIO based elements
		shooterEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k1X);
		climberSprockectLS = new DigitalInput(2);
		rightClimberLS = new DigitalInput(3);
		leftClimberLS = new DigitalInput(4);
		bottomGearLS = new DigitalInput(5);
		topGearLS = new DigitalInput(6);

		// Determine which motors are inverted empirically
		frontRight.setInverted(true);
		backRight.setInverted(false);
		frontLeft.setInverted(true);
		backLeft.setInverted(false);
		ropeClimberMotor.enableBrakeMode(true);

		// Initialize navX MXP to be primary gyroscope
		gyro = new AHRS(SerialPort.Port.kMXP);

		// Initialize controllers to correct ports
		pDriver = new XboxController(0);
		sDriver = new XboxController(1);

		// Initialize all necessary systems and mechanisms
		drive = new MecanumDrive(frontRight, backRight, frontLeft, backLeft, gyro, DRIFT_CONSTANT);
		shooter = new BallShooter(ballShooterMotor, agitator, shooterEncoder);
		ropeClimber = new RopeClimber(ropeClimberMotor, leftClimberLS, rightClimberLS);
		gearMechanism = new GearMechanism(gearMechanismMotor, bottomGearLS, topGearLS);
		macros = new Macros(gyro, CAMERA_X, CAMERA_Y, drive, shooter, gearMechanism, new XboxController[] { pDriver, sDriver });
		mech = new MechanismControls(shooter, gearMechanism, ropeClimber, macros);

		// Initialize booleans for field oriented toggle
		fieldOriented = false;
		previouslyPressed = false;
	}

	@Override
	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			// Determine if driver requests field-oriented driving or robot
			// respective driving.
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
		while (isAutonomous() && isEnabled()) {
			System.out.println(gyro.getDisplacementX());
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
}