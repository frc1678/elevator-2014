#include "CitrusButton.h"
#include "Coefficients.h"
#include "ControlLoop.h"
#include "CrioFile.h"
#include "Drivetrain.h"
#include "ElevatorSystem.h"
#include "MPU6050_I2C.h"
#include "NetworkTables/NetworkTable.h"
#include "WPILib.h" 	  //Lots of convenient robot classes



/*
 * This will be the project for the 2014 off season elevator robot
 * The robot has a flipped belly pan, and a cascade elevator mounted on top
 * 
 * The most important coding part of this robot is the control loops that should be invloved
 * They need to get invloved
 * 
 * Last Update date: 12/13/2014
 */






class RobotDemo : public IterativeRobot {
	
	DriverStation 	 *driverStation; 	// The drivestation console 
	DriverStationLCD *driverStationLCD;
	
	RobotDrive *drivetrain; // robot drive system

	Encoder *leftEncoder;	// Counts encoder clicks for the left and right sides of the drivetrain
	Encoder *rightEncoder;
	Encoder *elvEncoder;
	
	// MAP TESTING 
	map<string, float> constants; // For the coefficients from the FTP
	map<string, float> autoConstants;
	
	Solenoid *gearUp;
	Solenoid *gearDown;

	Joystick *manipulator;
	Joystick *driverL;
	Joystick *driverR;
	
	
	//Compressor
	Compressor *compressor;
		
	MPU6050_I2C *gyro;

	
	bool intakeDeployed;
	bool gearToggle;
	bool allDone;
	
		
	ElevatorSystem *cascadeElevator;
	
	CrioFile *elevatorLog;
	
//	
	// Buttons!
	CitrusButton *elevatorBottom; // Goes all the way to the ground
	CitrusButton *elevatorTop; // Goes all the way to the top
	CitrusButton *elevatorMid; // Goes to the center magnets
	CitrusButton *manualAdjust;
	CitrusButton *b_gearUp;
	CitrusButton *b_gearDown;
	
	
public:
	RobotDemo(){
		
		// This is the driverstation console. You can print to it if you want
		driverStation 	 = DriverStation::GetInstance();
		driverStationLCD = DriverStationLCD::GetInstance();
		
		
		// The drivetrain is the base of the robot that controls the wheels. 
		// 3 and 4 are the talons (speed controllers) that control the motors
		// that are attached to the drivetrain. The control how fast the robot drives
		// 3 is for the left side, 4 is for the right side. 
		drivetrain  = new RobotDrive(3, 1),	// these must be initialized in the same order

		
		leftEncoder  = new Encoder(4, 5); 
		rightEncoder = new Encoder(7, 6); // 7, 6 for the elevator
		elvEncoder   = new Encoder(3, 2);  
		
		//Begins counting of encoder clicks for both sides of the drivetrain
		leftEncoder->Start();
		rightEncoder->Start();
		elvEncoder->Start();
		
		// Solenoids = Control compressed air.
		// Control the gear shifting for the robot's drivetrain. This robot
		// will only ever be in low gear, but both are used to ensure that
		gearUp   = new Solenoid(8);
		gearDown = new Solenoid(7);
				
		driverL = new Joystick(1); //Joystick used to control left drivetrain
		driverR = new Joystick(2); //Joystick used to control right drivetrain
		manipulator = new Joystick(3); //Gamepad used to control most pneumatics and intakes



		// The compressor compresses the air used for deploying the intake.
		// The air is then stored in the tanks on board. 
		compressor = new Compressor(1, 1);
		
		

		
		
		// This boolean controls the gear toggle between low and high. 
		// For this robot, it will always be on low gear. 
		gearToggle = false;
		
		// Is the control loop for auto routines.
		allDone = false; 
		
			
		gyro = new MPU6050_I2C(constants, autoConstants);

		cascadeElevator = new ElevatorSystem (5, 7, 8, 9, elvEncoder, constants); // TODO ports obviously and change encoder
		
		elevatorLog = new CrioFile();	
		
		elevatorBottom = new CitrusButton(manipulator, 3);
		elevatorTop    = new CitrusButton(manipulator, 1);
		elevatorMid    = new CitrusButton(manipulator, 2);
		manualAdjust   = new CitrusButton(manipulator, 6);
		b_gearUp  	   = new CitrusButton(driverL, 2);
		b_gearDown	   = new CitrusButton(driverR, 2);
		
	}
	
	
	
	
	
	
	void UpdateButtons() 
	{
		elevatorBottom->Update();
		elevatorTop->Update();
		elevatorMid->Update();
		manualAdjust->Update();
		b_gearDown->Update();
		b_gearUp->Update();
	}
	
	
	
	
	
	
	
	
	void DisabledInit ()
	{
		leftEncoder->Reset();
		rightEncoder->Reset();  
		elvEncoder->Reset();
		
		compressor->Stop();
		
		driverStationLCD->Clear();
		driverStationLCD->UpdateLCD();
		drivetrain->TankDrive(0.0, 0.0);
		

		
		elevatorLog->EndLog();
		
		gyro->Stop();  
		gyro->Reset();
	}
	
	void DisabledPeriodic () {
		
		//cascadeElevator->CalibrateHalls();
		cascadeElevator->StopElevator();
	}
	
	
	
	
	
	
	
	void AutonomousInit()
	{
//		cascadeElevator->override = true;
//		if (driverStation->GetDigitalIn(1)){ cascadeElevator->UsePresets();}
//		else if (cascadeElevator->calibrationDone){cascadeElevator->UseCalibrated();}
//		else { cascadeElevator->UsePresets();}
//		 
		
		// Pull values from the log
		elevatorLog->StoreAutoValues(&autoConstants);
		
		// Start encoders at 0.0
		rightEncoder->Reset();
		leftEncoder->Reset();
		
		driverStationLCD->Clear(); // Clears all user messages from the driver console
		driverStationLCD->UpdateLCD(); // Must update for changes to the driver console to apply
		 
		gearUp->Set(false);  // Will stay this way throughout the match
		gearDown->Set(true); // Start in low gear
		
		//gyro->Stop();
		//gyro->Reset();
		
		bool allDone = false; //for continuing the while loop
			
		float leftMotorSpeed = 0.0;
		float rightMotorSpeed = 0.0;
		
			
		//autoTimer->Start();
		while((!allDone) && EnabledInAutonomous(this))
		{
			
			
			
			//first. Just driving forward to a point
			if(rightEncoder->Get() > -3000)//DriveForwardShootAutoConditions(timer, me, rightDT))
			{
				
				//drivetrain->TankDrive((-0.7)*leftMotorSpeed, (-0.7)*rightMotorSpeed);
				
				//drift to the right
//				if(gyro->GetFilterAngle() < 0) {
//					//rightMotorSpeed = -0.8;
//					rightMotorSpeed *= (-0.7); 
//					drivetrain->TankDrive(-0.7, rightMotorSpeed);
//				}
//				//drift to the left
//				else if(gyro->GetFilterAngle() > 0) {
//					//leftMotorSpeed = -0.8;
//					leftMotorSpeed *= (-0.7);
//					drivetrain->TankDrive(leftMotorSpeed, -0.7);
//				}
//				else {
//					leftMotorSpeed = -0.7;
//					rightMotorSpeed = -0.7;
//					drivetrain->TankDrive(-0.70, -0.70);
//				}
				//DriveForwardAutoInLoop(drivetrain); //Driving at almost full speed
			}
			
			else if(!allDone)
			{
				drivetrain->TankDrive(0.0, 0.0);
				allDone = true;
				break;
			}
			
			//driverStationLCD->Printf((DriverStationLCD::Line) 0, 1,
				//	"Count: %f", counter);
//			driverStationLCD->Printf((DriverStationLCD::Line) 1, 1,
//					"Angle: %f", gyro->GetGyroAngle());
//			driverStationLCD->Printf((DriverStationLCD::Line) 2, 1,
//					"Filter: %f", gyro->GetFilterAngle());
//			driverStationLCD->Printf((DriverStationLCD::Line) 3, 1,
//					"Rate: %f", gyro->GyroRate());
			driverStationLCD->Printf((DriverStationLCD::Line) 4, 1,
					"L Motor: %f", leftMotorSpeed);
			driverStationLCD->Printf((DriverStationLCD::Line) 5, 1,
					"R Motor: %f", rightMotorSpeed);
			driverStationLCD->UpdateLCD();
			
		}
		
	}
	
	void AutonomousPeriodic () {
		
	}

	
	
	
	
	
	
	void TeleopInit()
	{
		// For the first loop through. This is stuff we only want to run once.
//		if (driverStation->GetDigitalIn(1) || !cascadeElevator->calibrationDone) {
//			cascadeElevator->UsePresets();
//		}
//		else if (cascadeElevator->calibrationDone) {
//			cascadeElevator->UseCalibrated();
//		}
//		else {
//			cascadeElevator->UsePresets();
//		}
		
		gyro->Stop();
		gyro->Reset();
		elevatorLog->EndLog();
		elevatorLog->StartLog();
		
		// This sets the robot into low gear. Will not change throughout the match. TODO double check low gear
		gearToggle = false;
		gearUp->Set(gearToggle);  	
		gearDown->Set(!gearToggle);
	
		leftEncoder->Reset();
		rightEncoder->Reset();
		elvEncoder->Reset();
		
		// The compressor starts running and compresses air. Will take a little while 
		// to be fully charged, so fill them before a match. 
		compressor->Start();
		
		elevatorLog->StoreStartValues(&constants); 
	
	}
	
	
	void TeleopPeriodic () {
		// A tank drive on the gamepad
		//runDrivetrain(manipulator->GetRawAxis(2), manipulator->GetRawAxis(5), drivetrain);
		//runDrivetrainShift(driverL->GetY(), driverR->GetY(), drivetrain, 0.2, gearUp, gearDown, leftEncoder, rightEncoder);
		//drivetrain->TankDrive(0.0, 0.0);
		
		
		
		
		//elevatorLog->LogElevator(cascadeElevator);
		elevatorLog->LogEncoders(leftEncoder, rightEncoder);
		
		
		
		
		// Prints the encoder values to the driverstation
		driverStationLCD->Printf((DriverStationLCD::Line) 3, 1,
				"LE: %f", leftEncoder->GetRate());
		driverStationLCD->Printf((DriverStationLCD::Line) 4, 1,
				"RE: %f", rightEncoder->Get());
		driverStationLCD->Printf((DriverStationLCD::Line) 2, 1, "Gyro: %f", gyro->GetGyroAngle());
		driverStationLCD->Printf((DriverStationLCD::Line) 5, 1, "EE: %f", cascadeElevator->CurrentSpot());
		//driverStationLCD->Printf((DriverStationLCD::Line) 1, 1, "kpError: %f", allCoefficients->elvErr);
		//driverStationLCD->Printf((DriverStationLCD::Line) 0, 1, "Time: %f", currentTime);
		driverStationLCD->Printf((DriverStationLCD::Line) 0, 1, "Constants Check: %f", constants["ELV_KP"]);
		//driverStationLCD->Printf((DriverStationLCD::Line) 1, 15,	"kiError: %f ", allCoefficients->elvInt);	
		driverStationLCD->UpdateLCD();
		
		
		
		
		
		// GEARS
		// Gearshifting buttons. Are named accordingly
		if (b_gearUp->ButtonClicked()) {
			// Gear up
			gearUp->Set(false);
			gearDown->Set(true);
		}
		if (b_gearDown->ButtonClicked()) {
			// Gear down
			gearUp->Set(true);
			gearDown->Set(false);
		}
		

		
//		// Wanting the elevator to go DOWN and be SHORT
//		if (elevatorBottom->ButtonPressed() && !cascadeElevator->AtBottom()) {
//			cascadeElevator->RunElevatorDown(0);
//		}
//		// Wanting the elevator to be IN THE MIDDLE and be HALFWAY
//		else if (elevatorMid->ButtonPressed()) {
//			
//			// If elevator is BELOW the middle magnet, closer to the last magnet and the top of the elevator
//			// This means it needs to move DOWN to reach the middle
//			if (cascadeElevator->CurrentSpot() > cascadeElevator->correctHalls[2].highClicks) {
//				cascadeElevator->RunElevatorDown(2);	
//			}
//			// If the elevator is ABOVE the middle magnet, making it closer to the first magnet, and the bottom of the elevator
//			// The elevator needs to move UP to reach the middle
//			else if (cascadeElevator->CurrentSpot() < cascadeElevator->correctHalls[2].highClicks) {
//				cascadeElevator->RunElevatorUp(2);
//			}
//		}
//		// Wanting the elevator to go UP and be TALLLL
//		else if (elevatorTop->ButtonPressed() && !cascadeElevator->AtTop()) {
//			cascadeElevator->RunElevatorUp(0);
//		}
//		
//		if (manualAdjust->ButtonPressed()) {
//			cascadeElevator->SetElevatorSpeed(manipulator->GetRawAxis(2));
//		}
//		
		
		UpdateButtons();
	}
	
	
	
	void TestInit() {
		// Add test code later. If you want.
		// It's specific only to test though, so it won't overlap with auto and teleop
	}
	
	void TestPeriodic () {
		
	}
	
	bool EnabledInAutonomous(IterativeRobot *me)
	{
		if (me->IsAutonomous() && !me->IsDisabled())
		{
			return true;
		}
		return false;
	}
	
	
};

START_ROBOT_CLASS(RobotDemo);

