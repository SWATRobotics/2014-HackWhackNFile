#include "WPILib.h"


class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	DigitalInput shoot;
	Joystick stick; // only joystick
	Victor shooter;
	Victor forklift;
	Victor  winch;
	DoubleSolenoid shooterSole;//Festo
	Compressor compressor;
	Encoder shooterEnconder;
	AxisCamera *camera;
	
public:
	RobotDemo(): 
		myRobot(1,2,3,4),	//front left, rear left, front right, rear right	(10,4,3,7) for whack		(purple, yellow, green, red)
		shoot(10),
		stick(1),		// usb number
		shooter(6),
		forklift(5),
		winch(7),
		shooterSole(2,3),//initialize festo valves
					//forward channel, reverse channel
		compressor(3, 3),
		
			//(pressure switch channel, compressor relay channel) or
				//(pressure switch module number, pressure switch channel, compressor relay module number, compressor relay channel)
		shooterEnconder(5,6,false)//uint32_t aChannel, uint32_t bChannel, bool reverseDirection=false, EncodingType encodingType=k4X
	{
		myRobot.SetExpiration(0.1);
		camera = AxisCamera::GetInstance();
	}

	
	void Autonomous()
	{
		compressor.Start();//start compressor
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
		Wait(2.0); 				//    for 2 seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot
		
		
		while (!shoot.Get()){//while shooter isnt cocked pull it back
			shooter.SetSpeed(1); //set motor
		}
		
		shooterSole.Set(shooterSole.kForward);//Sets Solenoid forward, shoot ball
		shooterSole.Set(shooterSole.kReverse);//Sets Solenoid backward
		
		myRobot.Drive(1.0, -1.0);//turn around for 0.5 seconds, we have to check to see if it takes that long
		Wait(0.5);//wait for 0.5 seconds
		myRobot.Drive(0.0, 0.0);//stop robot
	}

	
	void OperatorControl()
	{
		compressor.Start();
		myRobot.SetSafetyEnabled(true);
		myRobot.SetInvertedMotor(myRobot.kRearLeftMotor, true);//Invert rear left Motor
		myRobot.SetInvertedMotor(myRobot.kRearRightMotor, true);//Invert rear right motor
		myRobot.SetInvertedMotor(myRobot.kFrontLeftMotor, true);//Invert rear right motor
		myRobot.SetInvertedMotor(myRobot.kFrontRightMotor, true);
		
		DriverStation *ds;
		DriverStationLCD *dsLCD;
		ds = DriverStation::GetInstance();
		dsLCD = DriverStationLCD::GetInstance();
		
		dsLCD->Printf(DriverStationLCD::kUser_Line1,1, "Starting Teleop");
		dsLCD->UpdateLCD();
		
		while (true)
		{
			if (!compressor.GetPressureSwitchValue()){
				compressor.Start();
			}
			myRobot.ArcadeDrive(stick); 
			
			 /*PNEUMATIC CODE*/
			if (stick.GetRawButton(8)){
				shooterSole.Set(shooterSole.kForward);
			}
			if (stick.GetRawButton(1)){
				shooterSole.Set(shooterSole.kReverse);
			}
			 /*SHOOTER CODE*/
			if (stick.GetRawButton(2)){
				shooter.SetSpeed(1);
			}
			if (stick.GetRawButton(4)){
				shooter.SetSpeed(-1);
				dsLCD->Printf(DriverStationLCD::kUser_Line1,(int)forklift.Get(), "Shooter Should be negative");
				dsLCD->UpdateLCD();
			}
			if (!stick.GetRawButton(4) || !stick.GetRawButton(2)){
				shooter.SetSpeed(0);
			}
			 /* FORKLIFT CODE*/
			if (!stick.GetRawButton(5) || !stick.GetRawButton(6)){
				forklift.SetSpeed(0);
			} 
			if (stick.GetRawButton(5)){
				forklift.SetSpeed(1);
			}
			if (stick.GetRawButton(6)){
				forklift.SetSpeed(-1);
				dsLCD->Printf(DriverStationLCD::kUser_Line1,(int)forklift.Get(), "Forklift Should be negative");
				dsLCD->UpdateLCD();
			}
			if (!shoot.Get()){
				shooter.SetSpeed(0);
				shooterSole.Set(shooterSole.kForward);
			}
			
			
			if (stick.GetRawButton(11)){
				//myRobot.m_rearLeftMotor.SetSpeed(1);
				//myRobot.m_rearRightMotor.SetSpeed(1);
				//myRobot.m_frontLeftMotor.SetSpeed(1);
				//myRobot.m_frontRightMotor.SetSpeed(1);
			}
			//Wait(0.005);// wait for a motor update time
		}
	}

	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);

