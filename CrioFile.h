#include "WPILib.h"
#include "MPU6050_I2C.h"
#include "ElevatorSystem.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <ctime>

#ifndef CRIOFILE_H //Include Gaurds
#define CRIOFILE_H

/*                                                                                                                                                                                                                                                                                                                                                                                                   
We use this to log various in match data to files on the cRIO. These files can be accessed by 
FTPing to them (from a computer commected to the robot).
*/

using namespace std;

class CrioFile {
	Timer *currentTimer;
	ofstream fileSave; //create new output file
	ifstream fileRead;
	bool currentlyLogging;
	float voltageCounter;
	
	
public:
	
	
	
	CrioFile()  
	{
		currentTimer = new Timer();
		currentlyLogging = false;
		voltageCounter = 0;
		
	}
	
	void StoreStartValues (map<string, float> *constants) {
		
		fileRead.open("coefficients.txt");
		//(*constants)["ELV_KP"] = 8;
		string key;
		float value;
		while (fileRead >> key >> value){
			(*constants)[key] = value;
		}		
		fileRead.close();
	}
		
	
	void StoreAutoValues (map<string, float> *constants) {
		fileRead.open("autodist.txt");
		string key;
		float value;
		while (fileRead >> key >> value){
			(*constants)[key] = value;
		}		
		fileRead.close();
	}
	
	
	void StartLog()
	{
		
		fileSave.open("File.txt"); //opening the file
		//reseting and starting the timer
		currentTimer->Reset(); 
		currentTimer->Start();
		
		currentlyLogging = true;
		fileSave<<"Time: "<<"Current:\n\r"; //writing to the file
		fileSave.close(); //Close the file
		fileSave.open("Heat.txt"); //Open DIFFERENT file
		currentlyLogging = true;
		fileSave<<"Time: "<<"Heat:\n\r"; //logging
		fileSave.close(); //Close it
		
		fileSave.open("Encoders.txt"); //Open wheel encoder file
		currentlyLogging = true;
		fileSave<<"Time: Rate: Clicks: \n\r"; //log
		fileSave.close(); //close
		
		fileSave.open("GyroAuto.txt");
		fileSave<<"Time: "<<"Angle: Accel: XAccel: YAccel: XOff: YOff: \n\r"; 
		fileSave.close();
		
		fileSave.open("GyroTele.txt");
		fileSave<<"Time: "<<"Angle: Accel: XAccel: YAccel: XOff: YOff: \n\r"; 
		fileSave.close();
		
		fileSave.open("ElevatorLog.txt");
		fileSave<<"Time: Encoders: TopTrig: BotTrig: YAccel: \n\r";
		fileSave.close();
		//Three seperate files, one for heat, one for rotation and one for current.
	}
	void LogCurrent(AnalogChannel *a)
	{
		//static ifstream fileIn; //create file object 
		//fileIn.open("File.txt"); //read in file
		
		/*bool timing = currentTimer->m_running;
		
		if(timing == false)
		{
			currentTimer->Start();
			currentTimer->Reset();			
		}*/

		if(currentlyLogging) //only run if we are logging
		{
			fileSave.open("File.txt", ios::app); //temp

			float current = a->GetVoltage();
			current = fabs(5-current); //log
			//Absolute value of 5 - current reading, the scale is inverted, this makes it easier to read		
			double time = currentTimer->Get();
			fileSave<<time<<" "<<current<<"\r\n"; //log two colomns, one with time and the other with current
			
			if(fileSave.fail())//test to see if file opened
			{
				printf("File did not open.\n");
			}
		}

		fileSave.close();
	}

	void LogHeat(AnalogChannel *b)
		{

			

			if(currentlyLogging) //only if we are currently logging
			{
				fileSave.open("Heat.txt", ios::app); //temp
				//app appends the writing, so it writes to the end of the file
				
				float Value = b->GetValue();
				double time = currentTimer->Get();
				fileSave<<time<<" "<<Value<<"\r\n";
				
				if(fileSave.fail())//test to see if file opened
				{
					printf("File did not open.\n");
				}
			}

			fileSave.close();
		}
	
	void LogEncoders(Encoder *leftEncoder, Encoder *rightEncoder)
	{
		
				
		if(currentlyLogging)
		{
			fileSave.open("Encoders.txt", ios::app);
			
			double time = currentTimer->Get();
			float rightRate = rightEncoder->GetRate();
			float leftRate = leftEncoder->GetRate();
			float rightValue = rightEncoder->Get();
			float leftValue = leftEncoder->Get();
			
			
			fileSave<<time<<" "<<leftRate<< " "<<rightRate<<"  "<<leftValue<< " "
					<<rightValue<<"\r\n"; //logging rotation rate and amount rotated for both sides of the drive train
			
			if(fileSave.fail())
			{
				printf("File did not open.\n");
			}
		}
		
		fileSave.close();
	}

	
	void LogGyroTele(MPU6050_I2C *gyro)
	{
			
					
			if(currentlyLogging)
			{
				fileSave.open("GyroTele.txt", ios::app);
				
				double time = currentTimer->Get();
				float angle = gyro->GetGyroAngle();
				//float rate = gyro->GyroRate();
				float XAccel = gyro->AccelRead(0) - gyro->GetXOffset();
				float YAccel = gyro->AccelRead(1) - gyro->GetYOffset();
				float XOff = gyro->GetXOffset();
				float YOff = gyro->GetYOffset();
				float Accel = gyro->GetAccel();
				
				fileSave<<time<<" "<<angle<<" "<<Accel<<" "<<XAccel<<" "<<YAccel<<" "<<XOff<<" "<<YOff<<"\r\n"; 
				//logging rotation rate and amount rotated for both sides of the drive train
				
				if(fileSave.fail())
				{
					printf("File did not open.\n");
				}
			}
			
			fileSave.close();
	}
	

	void LogGyroAuto(MPU6050_I2C *gyro)
	{
			
					
		if(currentlyLogging)
		{
			fileSave.open("GyroAuto.txt", ios::app);
			
			double time = currentTimer->Get();
			float angle = gyro->GetGyroAngle();
			//float rate = gyro->GyroRate();
			float XAccel = gyro->AccelRead(0) - gyro->GetXOffset();
			float YAccel = gyro->AccelRead(1) - gyro->GetYOffset();
			float XOff = gyro->GetXOffset();
			float YOff = gyro->GetYOffset();
			float Accel = gyro->GetAccel();
			
			fileSave<<time<<" "<<angle<<" "<<Accel<<" "<<XAccel<<" "<<YAccel<<" "<<XOff<<" "<<YOff<<"\r\n"; 
			//logging rotation rate and amount rotated for both sides of the drive train
			
			if(fileSave.fail())
			{
				printf("File did not open.\n");
			}
		}
		
		fileSave.close();
	}
	
	
	void LogElevator(ElevatorSystem *elevator) {
		
		if(currentlyLogging){
			fileSave.open("ElevatorLog.txt", ios::app);
			
			double time = currentTimer->Get();
			float encoder = elevator->CurrentSpot();
			bool topTriggered = elevator->TopHallTriggered();
			bool botTriggered = elevator->BottomHallTriggered();
			
			fileSave<<time<<" "<<encoder<<" "<<topTriggered<<" "<<botTriggered<<" \r\n";
					
			if(fileSave.fail()){
				printf("File ElevatorLog did not open \n");
			}
		}
		
		fileSave.close();
	}
	

	void EndLog()
	//flushing and closing the file
	{
		if(currentlyLogging)
		{
			currentlyLogging = false;
			fileSave.close();
			currentTimer->Stop();
		}
	}
	
	float CheckVoltage(AnalogChannel *a) //checking the voltage of the sensors
	{
		float current = a->GetVoltage();
		current = fabs(5.0-current); //check
		//inverse chart, therefore absolute value
		//5 b/c 0 = 5 on the inverted scale
		
		return current;
	}
	
	float CheckHeat(AnalogChannel *b) //checking the temperatue of the sensors
	{
		float Value = b->GetValue(); //This get function may end up being a different one.
		
		return Value;
	}
	
	void VoltageMonitor(Solenoid *gearUp, Solenoid *gearDown, CrioFile *currentSensor, 
			AnalogChannel *a, DriverStationLCD *driverStationLCD)
	//monitors the amount of voltage being drawn 
	{
		if(currentSensor->CheckVoltage(a) >= 3.5)
		{
			voltageCounter++;
			
			if(voltageCounter >= 15)
				//if it gets to high for enough time, auto shift down to low gear
				
			{
				gearUp->Set(false);
				gearDown->Set(true);
				voltageCounter = 0;
			}
		}
		else if(currentSensor->CheckVoltage(a) < 3.5)
		{
			voltageCounter = 0;
		}
		
		driverStationLCD->Printf((DriverStationLCD::Line) 5, 1, "Counter: %f", 
				voltageCounter);

		driverStationLCD->UpdateLCD();
		//printf("Counter: %f \n", voltageCounter);
	}
	/*
	void printFile()
	{
		string line;
		ifstream fileSave;
		if (fileSave.is_open())
		  {
		    while ( getline (fileSave,line) )
		    {
		      printf("%s\n",line);
		    }
		    fileSave.close();
		  }
		  else printf("Unable to open file"); 
	}
	*/
};

#endif
