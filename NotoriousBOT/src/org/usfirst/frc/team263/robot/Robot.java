package org.usfirst.frc.team263.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
	VictorSP gearMechanismMotor, hopperMotor;
	XboxController pDriver, sDriver;
	MecanumDrive drive;
	MechanismControls mech;
	BallShooter shooter;
	RopeClimber ropeClimber;
	GearMechanism gearMechanism;
	DigitalInput leftClimberLS, rightClimberLS, climberSprocketLS, bottomGearLS, topGearLS, cameraJumper;
	Encoder shooterEncoder;
	Macros macros;
	Autonomous autonomous;
	Servo servo;
	VisionProcessing visionProcessing;
	boolean fieldOriented, previouslyPressed;
	final double DRIFT_CONSTANT = 0.005;
	final int CAMERA_X = 360, CAMERA_Y = 240;

	@Override
	public void robotInit() {
		if (!cameraJumper.get()) {
			UsbCamera camera = new UsbCamera("cam0", 0);
			camera.setFPS(15);
			camera.setResolution(360, 240);
			CameraServer.getInstance().startAutomaticCapture();
		}
		LEDStrip.sendColor(LEDStrip.LEDMode.eRainbow);
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
		hopperMotor = new VictorSP(6);

		// Initialize all DIO based elements
		bottomGearLS = new DigitalInput(0);
		topGearLS = new DigitalInput(1);
		climberSprocketLS = new DigitalInput(2);
		rightClimberLS = new DigitalInput(3);
		leftClimberLS = new DigitalInput(4);
		cameraJumper = new DigitalInput(7);
		shooterEncoder = new Encoder(5, 6, false, Encoder.EncodingType.k2X);
		shooterEncoder.setReverseDirection(true);
		shooterEncoder.setSamplesToAverage(10);

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
		shooter = new BallShooter(ballShooterMotor, agitator, shooterEncoder, pDriver);
		ropeClimber = new RopeClimber(ropeClimberMotor, leftClimberLS, rightClimberLS);
		gearMechanism = new GearMechanism(gearMechanismMotor, bottomGearLS, topGearLS);
		macros = new Macros(gyro, CAMERA_X, CAMERA_Y, drive, shooter, gearMechanism,
				new XboxController[] { pDriver, sDriver });
		mech = new MechanismControls(shooter, gearMechanism, ropeClimber, macros, servo, hopperMotor);
		autonomous = new Autonomous(drive, gearMechanism, shooter, ropeClimber, climberSprocketLS);
		visionProcessing = new VisionProcessing(CAMERA_X, CAMERA_Y);

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
			
			// LEDStrip feedback logic
			if (gearMechanism.getState().equals(gearMechanism.getUp())) {
				
			}
		}
	}

	@Override
	public void autonomous() {
		gyro.zeroYaw();
		if (isAutonomous() && isEnabled()) {
			if (DriverStation.getInstance().getAlliance().equals(DriverStation.Alliance.Red)) {
				LEDStrip.sendColor(LEDStrip.LEDMode.eRed);
			} else {
				LEDStrip.sendColor(LEDStrip.LEDMode.eBlue);
			}
			String autoMode = CameraCoprocessor.getAutoMode();
			System.out.println(autoMode);
			if (autoMode.equals("Middle With Shot")) {
				autonomous.middleGearShoot();
			} else if (autoMode.equals("Left Gear Forward")) {
				autonomous.leftGear();
			} else if (autoMode.equals("Right Gear Forward")) {
				autonomous.rightGear();
			} else if (autoMode.equals("Middle Gear No Shot")) {
				autonomous.middleGear();
			} else if (autoMode.equals("Left Gear Still")) {
				autonomous.leftGearStill();
			} else if (autoMode.equals("Right Gear Still")) {
				autonomous.rightGearStill();
			} else if (autoMode.equals("Nothing")) {
			
			} else {
				System.out.println("Error - Recieved Unknown Command: " + autoMode);
			}
		}

	}

	@Override
	public void disabled() {
		super.disabled();
	}
}