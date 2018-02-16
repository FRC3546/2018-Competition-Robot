/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// TEAM 3546 - Buc'N'Gears
// (D)esign (O)riented (P)rogramming (E)nthusiast(S) (O)perating (S)ystem -> DOPES OS
// Version 1.0

#include <iostream>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "WPILib.h"
#include "AHRS.h"
#include "Joystick.h"
#include <LiveWindow/LiveWindow.h>
#include "ctre/Phoenix.h"

class Robot : public frc::IterativeRobot {

	// --------------------------------------------------------------------------------
	// PWM CHANNELS FOR DRIVETRAIN MOTORS
	const static int frontLeftChannel = 0;
	const static int rearLeftChannel = 1;
	const static int frontRightChannel = 2;
	const static int rearRightChannel = 3;
	// --------------------------------------------------------------------------------
	// JOYSTICK DEFINITIONS

	// Driver
	const static int joystickDriverUSBport = 0;
	const static int joystickDriver_Rotate_Axis = 2;		// for Mecanum Drive Rotation
	const static int zeroFieldAngleButton = 1;
	const static int rotateToFieldZeroButton = 2;
	const static int rotateToField90Button = 3;
	const static int rotateToField180Button = 4;
	const static int rotateToField270Button = 5;

	// CoDriver
	const static int joystickCoDriverUSBport = 1;
	const static int releasePowerCubeButton = 3;
	const static int intakePowerCubeButton = 1;
	const static int gripperUpDownToggleButton = 8;
	const static int gripperOpenButton = 2;
	const static int flipperEjectPowerCubeButton = 12;
	const static int platformRelease1Button = 5;
	const static int platformRelease2Button = 6;
	const static int platformExtendButton = 7;
	// --------------------------------------------------------------------------------
	// PNEUMATICS CONTROL MODULE DEFINITION
	const static int pcm0 = 0;		// CAN ID for PCM0
	const static int pcm1 = 1;		// CAN ID for PCM1

	const static int solenoidGripperUpDownPCM = pcm1;
	const static int solenoidGripperUpDownFwdChannel = 0;
	const static int solenoidGripperUpDownRvsChannel = 1;

	const static int solenoidGripperOpenClosePCM = pcm1;
	const static int solenoidGripperOpenCloseFwdChannel = 2;
	const static int solenoidGripperOpenCloseRvsChannel = 3;

	const static int solenoidFlipperPCM = pcm1;
	const static int solenoidFlipperFwdChannel = 4;
	const static int solenoidFlipperRvsChannel = 5;

	const static int solenoidPlatformReleasePCM = pcm1;
	const static int solenoidPlatformReleaseChannel = 6;

	const static int solenoidPlatformExtendPCM = pcm1;
	const static int solenoidPlatformExtendChannel = 7;
	// --------------------------------------------------------------------------------
	// GRIPPER MOTOR DEFINITION
	const static int gripperleftmotor = 2;		// CAN ID - Talon SRX
	const static int gripperightmotor = 1;		// CAN ID - Talon SRX
	// --------------------------------------------------------------------------------
	// SUBSYSTEM DEFINITION
	RobotDrive *robotDrive;		// Robot drive system
	Joystick *joystickDriver;	// Driver Joystick
	Joystick *joystickCoDriver;	// Co-Driver Joystick
	AHRS *ahrs;					// navX MXP
	Compressor *c;				// create compressor

	// GRIPPER MOTOR CONTROLLERS
	WPI_TalonSRX *gripperLeft;	// Left gripper motor controller
	WPI_TalonSRX *gripperRight;	// Right gripper motor controller

	// DOUBLE SOLENOIDS
	DoubleSolenoid *solenoidGR_UD;	// Solenoid Gripper Up-Down motion
	DoubleSolenoid *solenoidGR_OC;	// Solenoid Gripper Open-Close motion
	DoubleSolenoid *solenoidFL_ER;	// Solenoid Flipper Extend-Retract

	// SOLENOIDS
	Solenoid *solenoidPlatformRelease;
	Solenoid *solenoidPlatformExtend;

	// --------------------------------------------------------------------------------
public:
	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		// Define Robot Drivetrain
		robotDrive = new frc::RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);     // initialize variables in
		robotDrive->SetExpiration(0.1);

		// Invert Motors on the Robot's RIGHT side
		robotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);

		// Define Joysticks
		joystickDriver = new frc::Joystick(joystickDriverUSBport);                // same order declared above
		joystickCoDriver = new frc::Joystick(joystickCoDriverUSBport);

		// Define Compressor
		c = new frc::Compressor(pcm0);
		c->SetClosedLoopControl(true);        // turn on compressor, set to false to turn off

		// Define Gripper Motors
		gripperLeft = new WPI_TalonSRX(gripperleftmotor);
		gripperRight = new WPI_TalonSRX(gripperightmotor);

		// Define Gyro - NAVX Board
		ahrs = new AHRS(SPI::Port::kMXP);
		ahrs->ZeroYaw();

		// Define Solenoids/DoubleSolenoids
		solenoidGR_UD = new DoubleSolenoid(solenoidGripperUpDownPCM, solenoidGripperUpDownFwdChannel, solenoidGripperUpDownRvsChannel);
		solenoidGR_OC = new DoubleSolenoid(solenoidGripperOpenClosePCM, solenoidGripperOpenCloseFwdChannel, solenoidGripperOpenCloseRvsChannel);
		solenoidFL_ER = new DoubleSolenoid(solenoidFlipperPCM, solenoidFlipperFwdChannel, solenoidFlipperRvsChannel);
		solenoidPlatformRelease = new Solenoid(solenoidPlatformReleasePCM, solenoidPlatformReleaseChannel);
		solenoidPlatformExtend = new Solenoid(solenoidPlatformExtendPCM, solenoidPlatformExtendChannel);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {}

	void TeleopPeriodic()
	{
		robotDrive->SetSafetyEnabled(false);
		while (IsOperatorControl() && IsEnabled())
		{
			// --------------------------------------------------------------------------------
			//Drive robot with Driver's joystick input
			robotDrive->MecanumDrive_Cartesian(joystickDriver->GetX(),
											   joystickDriver->GetY(),
											   joystickDriver->GetRawAxis(joystickDriver_Rotate_Axis)*0.7,
											   ahrs->GetAngle());

			// --------------------------------------------------------------------------------
			//Zero field orientation
			bool reset_yaw_button_pressed = joystickDriver->GetRawButton(zeroFieldAngleButton);
			if ( reset_yaw_button_pressed )
			{
				ahrs->ZeroYaw();
			}

			// --------------------------------------------------------------------------------
			// TURN GRIPPER MOTORS ON FOR POWER CUBE INTAKE OR RELEASE
			bool pressRelease = joystickCoDriver->GetRawButton(releasePowerCubeButton);
			bool pressIntake = joystickCoDriver->GetRawButton(intakePowerCubeButton);

			if (pressRelease && !pressIntake)		// Release Power Cube
			{
				gripperLeft->Set(-0.65);
				gripperRight->Set(0.65);
			}
			else if (!pressRelease && pressIntake)	// Intake Power Cube
			{
				gripperLeft->Set(0.4);
				gripperRight->Set(-0.4);
			}
			else									// ... or else, Stop Motors
			{
				gripperLeft->Set(0);
				gripperRight->Set(0);
			}

			// --------------------------------------------------------------------------------
			// TOGGLE GRIPPER UP-DOWN WITH CO-DRIVER JOYSTICK BUTTON 8
			bool gripper_updown = joystickCoDriver->GetRawButton(gripperUpDownToggleButton);
			if (gripper_updown)
			{
				if (solenoidGR_UD->Get()== 2)
				{
					solenoidGR_UD->Set(DoubleSolenoid::Value::kForward);
					Wait(1);
				}
				else
				{
					solenoidGR_UD->Set(DoubleSolenoid::Value::kReverse);
					Wait(1);
				}
			}

			// --------------------------------------------------------------------------------
			// OPEN GRIPPER WHILE HOLDING BUTTON
			bool gripperOpen = joystickCoDriver->GetRawButton(gripperOpenButton);
			if ( gripperOpen )
			{
				solenoidGR_OC->Set(DoubleSolenoid::Value::kReverse);
			}
			else
			{
				solenoidGR_OC->Set(DoubleSolenoid::Value::kForward);
			}

			// --------------------------------------------------------------------------------
			// EJECT POWER CUBE USING FLIPPER WITH BUTTON 12
			bool flipperEject = joystickCoDriver->GetRawButton(flipperEjectPowerCubeButton);
			if ( flipperEject )
			{
				solenoidGR_OC->Set(DoubleSolenoid::Value::kReverse);
				Wait(1);
				solenoidFL_ER->Set(DoubleSolenoid::Value::kForward);
				Wait(0.5);
				solenoidFL_ER->Set(DoubleSolenoid::Value::kReverse);
				Wait(1);
				solenoidGR_OC->Set(DoubleSolenoid::Value::kForward);
			}

			// --------------------------------------------------------------------------------
			// CHECK FOR PLATFORM RELEASE
			bool platformRelease1 = joystickCoDriver->GetRawButton(platformRelease1Button);
			bool platformRelease2 = joystickCoDriver->GetRawButton(platformRelease2Button);

			if (platformRelease1 && platformRelease2)
			{
				solenoidPlatformRelease->Set(true);	// release platform
			}

			// --------------------------------------------------------------------------------
			// CHECK for PLATFORM EXTEND
			bool platformExtend = joystickCoDriver->GetRawButton(platformExtendButton);
			if (platformExtend && solenoidPlatformRelease->Get() == false)
			{
				solenoidPlatformExtend->Set(true);	// extend platform
			}
			// --------------------------------------------------------------------------------

			Wait(0.05); // wait 50ms to avoid hogging CPU cycles
		}
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
