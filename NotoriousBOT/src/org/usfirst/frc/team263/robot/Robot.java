package org.usfirst.frc.team263.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
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
	DigitalInput leftClimberLS, rightClimberLS, climberSprocketLS, bottomGearLS, topGearLS, cameraJumper;
	Encoder shooterEncoder;
	Macros macros;
	Autonomous autonomousThread;
	Servo servo;
	boolean fieldOriented, previouslyPressed;
	final double DRIFT_CONSTANT = 0.005;
	final int CAMERA_X = 640, CAMERA_Y = 480;
	
	@Override
	public void robotInit() {
		if (cameraJumper.get()) {
			UsbCamera camera = new UsbCamera("cam0", 0);
			camera.setFPS(15);
			camera.setResolution(360, 240);
			CameraServer.getInstance().startAutomaticCapture();
		}
	}

	public Robot() {
		// Initialize motor controller addresses
		ballShooterMotor = new CANTalon(0);
		ropeClimberMotor = new CANTalon(1);
		frontLeft = new VictorSP(0);
		backLeft = new VictorSP(1);
		frontRight = new VictorSP(2);
		backRight = new VictorSP(3);
		gearMechanismMotor = new VictorSP(4);
		agitator = new VictorSP(5);

		// Initialize all DIO based elements
		bottomGearLS = new DigitalInput(0);
		topGearLS = new DigitalInput(1);
		climberSprocketLS = new DigitalInput(2);
		rightClimberLS = new DigitalInput(3);
		leftClimberLS = new DigitalInput(4);
		cameraJumper = new DigitalInput(7);
		shooterEncoder = new Encoder(5, 6, false, Encoder.EncodingType.k1X);

		// Determine which motors are inverted empirically
		frontRight.setInverted(true);
		backRight.setInverted(true);
		frontLeft.setInverted(false);
		backLeft.setInverted(false);
		ballShooterMotor.setInverted(true);
		ropeClimberMotor.enableBrakeMode(true);

		// Initialize navX MXP to be primary gyroscope
		gyro = new AHRS(SerialPort.Port.kMXP);

		// Initialize controllers to correct ports
		pDriver = new XboxController(0);
		sDriver = new XboxController(1);
		
		servo = new Servo(9);
		
		// Initialize all necessary systems and mechanisms
		drive = new MecanumDrive(frontRight, backRight, frontLeft, backLeft, gyro, DRIFT_CONSTANT);
		shooter = new BallShooter(ballShooterMotor, agitator, shooterEncoder);
		ropeClimber = new RopeClimber(ropeClimberMotor, leftClimberLS, rightClimberLS);
		gearMechanism = new GearMechanism(gearMechanismMotor, bottomGearLS, topGearLS);
		macros = new Macros(gyro, CAMERA_X, CAMERA_Y, drive, shooter, gearMechanism, new XboxController[] { pDriver, sDriver });
		mech = new MechanismControls(shooter, gearMechanism, ropeClimber, macros, servo);
		autonomousThread = new Autonomous(drive);

		// Initialize booleans for field oriented toggle
		fieldOriented = false;
		previouslyPressed = false;
	}

	@Override
	public void operatorControl() {
		System.out.println("calibrated");
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
		if (isAutonomous() && isEnabled()) {
			//autonomousThread.start();
			drive.forward(0.5, 1350);
			gearMechanism.toggleState();
			gearMechanism.run();
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			drive.forward(-0.5, 600);
			gearMechanism.toggleState();
			gearMechanism.run();
		} 
		//autonomousThread.interrupt();
	}
	
	@Override
	public void disabled() {
		if (autonomousThread.isAlive()) {
			autonomousThread.interrupt();
		}
	}
}