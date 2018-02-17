/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// TEAM 3546 - Buc'N'Gears
// (D)esign (O)riented (P)rogramming (E)nthusiast(S) (O)perating (S)ystem -> DOPES OS
// Version 1.01

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

private:
	// --------------------------------------------------------------------------------
	// PWM CHANNELS FOR DRIVETRAIN MOTORS
	const static int frontLeftMotorPWMChannel = 0;
	const static int rearLeftMotorPWMChannel = 1;
	const static int frontRightMotorPWMChannel = 2;
	const static int rearRightMotorPWMChannel = 3;
	// --------------------------------------------------------------------------------
	// JOYSTICK DEFINITIONS

	// Driver
	const static int joystickDriverUSBport = 0;
	const static int joystickDriver_Rotate_Axis = 2;		// for Mecanum Drive Rotation
	const double  rotationScalingFactor = 0.7;				// scaling factor for rotation axis
	const static int zeroFieldAngleButton = 8;
	const static int rotateToFieldZeroButton = 2;
	const static int rotateToField90Button = 3;
	const static int rotateToField180Button = 4;
	const static int rotateToField270Button = 5;

	// CoDriver
	const static int joystickCoDriverUSBport = 1;
	const static int releasePowerCubeButton = 3;
	const static int intakePowerCubeButton = 1;
	const static int gripperDownButton = 8;
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

	// CREATING SENDABLE CHOOSER FOR AUTONOMOUS
	std::unique_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<frc::Command*> chooser;

	std::string gameData;
	int alliance;
	int location;

	// --------------------------------------------------------------------------------

public:

	// --------------------------------------------------------------------------------
	// GRIPPER MOTORS
	// --------------------------------------------------------------------------------
	void ReleasePowerCubeMotors(double motorSpeed)
	{
		gripperLeft->Set(-motorSpeed);
		gripperRight->Set(motorSpeed);
	}

	void IntakePowerCubeMotors(double motorSpeed)
	{
		gripperLeft->Set(motorSpeed);
		gripperRight->Set(-motorSpeed);
	}

	void StopPowerCubeMotors(void)
	{
		gripperLeft->Set(0);
		gripperRight->Set(0);
	}

	// --------------------------------------------------------------------------------
	// GRIPPER UP-DOWN POSITION
	// --------------------------------------------------------------------------------
	void LowerGripper(void)
	{
		solenoidGR_UD->Set(DoubleSolenoid::Value::kReverse);
	}

	void RaiseGripper(void)
	{
		solenoidGR_UD->Set(DoubleSolenoid::Value::kForward);
	}

	// --------------------------------------------------------------------------------
	// GRIPPER OPEN-CLOSE POSITION
	// --------------------------------------------------------------------------------

	void CloseGripper(void)
	{
		solenoidGR_OC->Set(DoubleSolenoid::Value::kForward);
	}

	void OpenGripper(void)
	{
		solenoidGR_OC->Set(DoubleSolenoid::Value::kReverse);
	}

	// --------------------------------------------------------------------------------
	// FLIPPER OPEN-CLOSE POSITION
	// --------------------------------------------------------------------------------
	void FlipperOpen(void)
	{
		solenoidFL_ER->Set(DoubleSolenoid::Value::kForward);
	}

	void FlipperClose(void)
	{
		solenoidFL_ER->Set(DoubleSolenoid::Value::kReverse);
	}

	// --------------------------------------------------------------------------------
	// PLATFORM RELEASE
	// --------------------------------------------------------------------------------
	void ReleasePlatform(double DelayInSeconds)
	{
		Wait(DelayInSeconds); // delay to ensure we really want the platform to be released

		bool platformRelease1 = joystickCoDriver->GetRawButton(platformRelease1Button);	// recheck button press
		bool platformRelease2 = joystickCoDriver->GetRawButton(platformRelease2Button);	// recheck button press

		// if both buttons are still pressed, then release platform
		if (platformRelease1 && platformRelease2)
		{
			solenoidPlatformRelease->Set(true);	// release platform
		}
	}
	// --------------------------------------------------------------------------------
	// PLATFORM EXTEND
	// --------------------------------------------------------------------------------
	void ExtendPlatform()
	{
		solenoidPlatformExtend->Set(true);	// extend platform
	}
	// --------------------------------------------------------------------------------
	// AUTONOMOUS COMMANDS
	// --------------------------------------------------------------------------------

	void DoNothing(void){};

	void Drive(double foreaft, double side2side, double time2drive)
	{
		robotDrive->MecanumDrive_Cartesian(side2side, foreaft, 0, ahrs->GetAngle());
		Wait(time2drive);
		//robotDrive->StopMotor();
		// or
		robotDrive->MecanumDrive_Cartesian(0, 0, 0, 0);
	}


	void RobotInit() {

		m_chooser.AddDefault("Do Nothing","Do Nothing");
		m_chooser.AddObject("Drive Fwd Only", "Drive Fwd Only");
		m_chooser.AddObject("Robot Time","Robot Time");
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		// Define Robot Drivetrain
		robotDrive = new frc::RobotDrive(frontLeftMotorPWMChannel,
				                         rearLeftMotorPWMChannel,
										 frontRightMotorPWMChannel,
										 rearRightMotorPWMChannel);
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

		// Set Initial States of Mechanisms
		StopPowerCubeMotors();
		RaiseGripper();
		CloseGripper();
		FlipperClose();
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

		// GET DATA TO SELECT AUTONOMOUS
		//alliance = frc::DriverStation::GetInstance().GetAlliance();				// either kRed or kBlue
		location = alliance = frc::DriverStation::GetInstance().GetLocation();	// 1 (left), 2 (middle), 3 (right)
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();	// randomized switch & scale colors

	}

	void AutonomousPeriodic() {

		if (m_autoSelected == "Do Nothing")
		{
			//DoNothing();
			while(IsAutonomous());
		}
		else if (m_autoSelected == "Drive Fwd Only")
		{
			Drive(-0.5, 0, 1);	// tune these values so that we will always cross the line
			while(IsAutonomous());
		}
		else if (m_autoSelected == "Robot Time")
		{
			//-------------------------------------------------------------------------------------------
			// IF STARTING POSITION NOT IN THE MIDDLE
			if (location == 1 || location == 3)
			{
				Drive(-0.5, 0, 1);	// tune these values so that we will always cross the line

				if (location == 1 && gameData[0] == 'L')
					ReleasePowerCubeMotors(0.75);

				if (location == 3 && gameData[0] == 'R')
					ReleasePowerCubeMotors(0.75);

				while(IsAutonomous());	// wait here until autonomous period ends
			}
			//-------------------------------------------------------------------------------------------
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
											   joystickDriver->GetRawAxis(joystickDriver_Rotate_Axis)*rotationScalingFactor,
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
				ReleasePowerCubeMotors(0.75);
			}
			else if (!pressRelease && pressIntake)	// Intake Power Cube
			{
				IntakePowerCubeMotors(0.3);
			}
			else									// ... or else, Stop Motors
			{
				StopPowerCubeMotors();
			}

			// --------------------------------------------------------------------------------
			// GRIPPER DOWN WHILE HOLDING DOWN CO-DRIVER JOYSTICK BUTTON 8
			bool gripperDown = joystickCoDriver->GetRawButton(gripperDownButton);
			if (gripperDown)
			{
				LowerGripper();	// Gripper in down position
			}
			else
			{
				RaiseGripper();	// Gripper in up position
			}

			// --------------------------------------------------------------------------------
			// OPEN GRIPPER WHILE HOLDING BUTTON
			bool gripperOpen = joystickCoDriver->GetRawButton(gripperOpenButton);
			if ( gripperOpen )
			{
				OpenGripper();
			}
			else
			{
				CloseGripper();
			}

			// --------------------------------------------------------------------------------
			// EJECT POWER CUBE USING FLIPPER
			bool flipperEject = joystickCoDriver->GetRawButton(flipperEjectPowerCubeButton);
			if ( flipperEject )
			{
				OpenGripper();
				Wait(1);
				FlipperOpen();
				Wait(0.5);
				FlipperClose();
				Wait(1);
				CloseGripper();
			}

			// --------------------------------------------------------------------------------
			// CHECK FOR PLATFORM RELEASE
			bool platformRelease1 = joystickCoDriver->GetRawButton(platformRelease1Button);
			bool platformRelease2 = joystickCoDriver->GetRawButton(platformRelease2Button);

			if (platformRelease1 && platformRelease2)
			{
				ReleasePlatform(2);	// release platform after button is held for 2 seconds
			}

			// --------------------------------------------------------------------------------
			// CHECK for PLATFORM EXTEND
			bool platformExtend = joystickCoDriver->GetRawButton(platformExtendButton);
			if (platformExtend && solenoidPlatformRelease->Get() == false)
			{
				ExtendPlatform();
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
