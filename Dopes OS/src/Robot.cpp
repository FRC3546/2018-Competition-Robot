// TEAM 3546 - Buc'N'Gears
// (D)esign (O)riented (P)rogramming (E)nthusiast(S) (O)perating (S)ystem -> DOPES OS
// Version 1.10

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
#include "Timer.h"

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
    const static int joystickDriver_Rotate_Axis = 2;    // for Mecanum Drive Rotation
    const double  rotationScalingFactor = 0.6;            // scaling factor for rotation axis
    const static int zeroFieldAngleButton = 8;

    // CoDriver
    const static int joystickCoDriverUSBport = 1;
    const static int releasePowerCubeButton = 3;
    const static int intakePowerCubeButton = 1;
    const static int gripperDownButton = 7;
    const static int gripperUpButton = 8;
    const static int gripperOpenButton = 2;
    const static int flipperEjectPowerCubeButton = 12;
    const static int platformRelease1Button = 9;
    const static int platformRelease2Button = 10;
    const static int platformExtendButton = 7;
    // --------------------------------------------------------------------------------
    // MOTOR SPEEDS
    const double motorReleasePowerCubeSpeed = 0.75;
    const double motorIntakePowerCubeSpeed = 0.3;
    const double motorAutonomousSpeed = 0.5;
    // --------------------------------------------------------------------------------
    // PNEUMATICS CONTROL MODULE DEFINITION
    const static int pcm0 = 0;        // CAN ID for PCM0

    const static int solenoidGripperUpDownPCM = pcm0;
    const static int solenoidGripperUpDownFwdChannel = 0;        // raise gripper
    const static int solenoidGripperUpDownRvsChannel = 1;        // lower gripper

    const static int solenoidGripperOpenClosePCM = pcm0;
    const static int solenoidGripperOpenCloseFwdChannel = 2;    // open gripper
    const static int solenoidGripperOpenCloseRvsChannel = 3;    // close gripper

    const static int solenoidFlipperPCM = pcm0;
    const static int solenoidFlipperFwdChannel = 4;                // flipper eject motion
    const static int solenoidFlipperRvsChannel = 5;                // flipper retract motion

    const static int solenoidPlatformPCM = pcm0;
    const static int solenoidPlatformFwdChannel = 6;            // platform engage
    const static int solenoidPlatformRvsChannel = 7;            // platform release

    // --------------------------------------------------------------------------------
    // GRIPPER MOTOR DEFINITION
    const static int gripperleftmotor = 2;        // CAN ID - Talon SRX
    const static int gripperightmotor = 1;        // CAN ID - Talon SRX
    // --------------------------------------------------------------------------------
    // SUBSYSTEM DEFINITION
    RobotDrive *robotDrive;        	// Robot drive system
    Joystick *joystickDriver;    	// Driver Joystick
    Joystick *joystickCoDriver;    	// Co-Driver Joystick
    AHRS *ahrs;                    	// navX MXP
    Compressor *c;                	// create compressor

    // GRIPPER MOTOR CONTROLLERS
    WPI_TalonSRX *gripperLeft;    	// Left gripper motor controller
    WPI_TalonSRX *gripperRight;    	// Right gripper motor controller

    // DOUBLE SOLENOIDS
    DoubleSolenoid *solenoidGR_UD;  // Solenoid Gripper Up-Down motion
    DoubleSolenoid *solenoidGR_OC;  // Solenoid Gripper Open-Close motion
    DoubleSolenoid *solenoidPLT;    // Solenoid Platform Engage-Release

    // DATA FROM FIELD MANAGEMENT SYSTEM
    std::string gameData;
    int location;

    // TIMER
    Timer *timer;
    int commandTimeout;

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
    // PLATFORM RELEASE
    // --------------------------------------------------------------------------------
    void ReleasePlatform(double DelayInSeconds)
    {
        Wait(DelayInSeconds); // delay to ensure we really want the platform to be released

        bool platformRelease1 = joystickCoDriver->GetRawButton(platformRelease1Button);    // recheck button press
        bool platformRelease2 = joystickCoDriver->GetRawButton(platformRelease2Button);    // recheck button press

        // if both buttons are still pressed, then release platform
        if (platformRelease1 && platformRelease2)
        {
            LowerGripper();
            solenoidPLT->Set(DoubleSolenoid::Value::kReverse);    // release platform
        }
    }
    // --------------------------------------------------------------------------------
    // PLATFORM EXTEND
    // --------------------------------------------------------------------------------
    void EngagePlatform()
    {
        solenoidPLT->Set(DoubleSolenoid::Value::kForward);    // engage platform
    }
    // --------------------------------------------------------------------------------
    // AUTONOMOUS COMMANDS
    // --------------------------------------------------------------------------------
    void MoveForward(double motorSpeed, double time2Drive)
	{
		timer->Reset();
		timer->Start();
		while (timer->Get()<time2Drive)
		{
			//robotDrive->MecanumDrive_Cartesian(side2side, foreaft, 0, ahrs->GetAngle());
			robotDrive->MecanumDrive_Cartesian(0, -motorSpeed, 0, ahrs->GetAngle());
		}
		robotDrive->MecanumDrive_Cartesian(0, 0, 0, ahrs->GetAngle());
		timer->Reset();
	}

    void MoveBackward(double motorSpeed, double time2Drive)
	{
		timer->Reset();
		timer->Start();
		while (timer->Get()<time2Drive)
		{
			robotDrive->MecanumDrive_Cartesian(0, motorSpeed, 0, ahrs->GetAngle());
		}
		robotDrive->MecanumDrive_Cartesian(0, 0, 0, ahrs->GetAngle());
		timer->Reset();
	}

    void MoveLeft(double motorSpeed, double time2Drive)
	{
		timer->Reset();
		timer->Start();
		while (timer->Get()<time2Drive)
		{
			robotDrive->MecanumDrive_Cartesian(-motorSpeed, -motorSpeed*0.5, 0, ahrs->GetAngle());
		}
		robotDrive->MecanumDrive_Cartesian(0, 0, 0, ahrs->GetAngle());
		timer->Reset();
	}

    void MoveRight(double motorSpeed, double time2Drive)
	{
		timer->Reset();
		timer->Start();
		while (timer->Get()<time2Drive)
		{
			robotDrive->MecanumDrive_Cartesian(motorSpeed, -motorSpeed*0.5, 0, ahrs->GetAngle());
		}
		robotDrive->MecanumDrive_Cartesian(0, 0, 0, ahrs->GetAngle());
		timer->Reset();
	}
    // --------------------------------------------------------------------------------

    void RobotInit() {

        m_chooser.AddDefault("Do Nothing","Do Nothing");
        m_chooser.AddObject("Left Start","Left Start");
        m_chooser.AddObject("Middle Start","Middle Start");
        m_chooser.AddObject("Right Start","Right Start");
        m_chooser.AddObject("Drive Forward Only","Drive Forward Only");
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
        solenoidPLT = new DoubleSolenoid(solenoidPlatformPCM, solenoidPlatformFwdChannel, solenoidPlatformRvsChannel);

        // Timer
        timer = new Timer;

        // Set Initial States of Mechanisms
        StopPowerCubeMotors();
        RaiseGripper();
        CloseGripper();
        EngagePlatform();
    }

    void AutonomousInit() override {

        // Get Data to Select Autonomous
    	m_autoSelected = m_chooser.GetSelected();
        gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();    // randomized switch & scale colors

        // Zero Gyro
        ahrs->ZeroYaw();
    }

    void AutonomousPeriodic() {

        if (m_autoSelected == "Do Nothing")
        {
            while(IsAutonomous());
        }
        else if (m_autoSelected == "Left Start" || m_autoSelected == "Middle Start" || m_autoSelected == "Right Start")
        {
            if (m_autoSelected == "Left Start")
                location = 1;

            if (m_autoSelected == "Middle Start")
                location = 2;

            if (m_autoSelected == "Right Start")
                location = 3;

            //-------------------------------------------------------------------------------------------
            // IF STARTING POSITION NOT IN THE MIDDLE
            if (location == 1 || location == 3)
            {
            	MoveForward(motorAutonomousSpeed, 1.0);    // tune these values so that we will always get to the switch
                Wait(1);

                if (location == 1)
                {
                    if (gameData[0] == 'L')
                    {
                    	ReleasePowerCubeMotors(motorReleasePowerCubeSpeed );
                    	Wait(1);
						StopPowerCubeMotors();
                    }
                }

                if (location == 3)
                {
                	if (gameData[0] == 'R')
                    {
                		ReleasePowerCubeMotors(motorReleasePowerCubeSpeed );
                		Wait(1);
                		StopPowerCubeMotors();
                    }
                }

                while(IsAutonomous());    // wait here until autonomous period ends
            }
            //-------------------------------------------------------------------------------------------
            // IF STARTING POSITION IS IN THE MIDDLE
            else
            {

                if (gameData[0] == 'L')    // go to the left
                {
                    //MoveForward(motorAutonomousSpeed, 0.7);		// drive forward a bit
                    MoveLeft(motorAutonomousSpeed, 2);		// drive left
                    MoveForward(motorAutonomousSpeed*0.5, 0.8);		// drive to the switch
                    //Wait(1);
					ReleasePowerCubeMotors(motorReleasePowerCubeSpeed);
					Wait(0.7);
					StopPowerCubeMotors();
                }
                else        // go to the right
                {
                    //MoveForward(motorAutonomousSpeed, 0.5);		// drive forward a bit
                    MoveRight(motorAutonomousSpeed, 1.3);		// drive right
                    MoveForward(motorAutonomousSpeed*0.5, 0.8);		// drive to the switch
                    Wait(0.7);
					ReleasePowerCubeMotors(motorReleasePowerCubeSpeed);
					Wait(1);
					StopPowerCubeMotors();
                }

                while(IsAutonomous());
            }

            //-------------------------------------------------------------------------------------------
        }
        else if (m_autoSelected == "Do Nothing")
        {
        	MoveForward(motorAutonomousSpeed, 1.0);
        	while(IsAutonomous());
        }
    }

    void TeleopInit()
    {
        // Set Initial States of Mechanisms
        StopPowerCubeMotors();
        RaiseGripper();
        CloseGripper();
        EngagePlatform();
    }

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

            if (pressRelease && !pressIntake)        // Release Power Cube
            {
                ReleasePowerCubeMotors(motorReleasePowerCubeSpeed );
            }
            else if (!pressRelease && pressIntake)    // Intake Power Cube
            {
                IntakePowerCubeMotors(motorIntakePowerCubeSpeed );
            }
            else                                    // ... or else, Stop Motors
            {
                StopPowerCubeMotors();
            }
            // --------------------------------------------------------------------------------
            // GRIPPER DOWN WHILE HOLDING DOWN CO-DRIVER JOYSTICK BUTTON 8
            bool gripperDown = joystickCoDriver->GetRawButton(gripperDownButton);
            bool gripperUp = joystickCoDriver->GetRawButton(gripperUpButton);

            if (gripperDown && !gripperUp)    // in button press lower mode
            {
                LowerGripper();    // Gripper in down position
            }
            else if (!gripperDown && gripperUp)
            {
                RaiseGripper();    // Gripper in up position
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
            // CHECK FOR PLATFORM RELEASE
            bool platformRelease1 = joystickCoDriver->GetRawButton(platformRelease1Button);
            bool platformRelease2 = joystickCoDriver->GetRawButton(platformRelease2Button);

            if (platformRelease1 && platformRelease2)
            {
                ReleasePlatform(0.5);    // release platform after button is held for 2 seconds
                //Wait(3);
                //EngagePlatform();
            }

            // --------------------------------------------------------------------------------
            // CHECK for PLATFORM Engage
            bool platformEngage = joystickCoDriver->GetRawButton(platformExtendButton);
            if (platformEngage)
            {
                EngagePlatform();
            }
            // --------------------------------------------------------------------------------

            Wait(0.001);
        }
    }

    void TestPeriodic() {}

private:
    frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
    frc::SendableChooser<std::string> m_chooser;
    std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)

