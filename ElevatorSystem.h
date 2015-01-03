#include "WPILib.h"
#include "Coefficients.h"
#include "ControlLoop.h"

#ifndef ELEVATORSYSTEM_H
#define ELEVATORSYSTEM_H


class ElevatorSystem {
	
	Talon *elvMotor1;
	Talon *elvMotor2;
	Encoder *elvEncoder;
	
	// Flags
	bool topTriggered; // To signal if the sensors are triggered
	bool bottomTriggered;
	bool highFilled; // To make sure both sensors were triggered
	bool lowFilled;
		
	// The two proxy sensors. One's above the other
	DigitalInput *topHallSensor;
	DigitalInput *bottomHallSensor;
	
	ControlLoop *elvPID;
	
	float kp, ki, kd;

	// For the hall sensors and at what encoder values they trigger. 
	map<string, float> *presetsLow;
	map<string, float> *presetsHigh;
	map<string, float> *calibratedLow;
	map<string, float> *calibratedHigh;
	map<string, float> *correctLow;
	map<string, float> *correctHigh;
	
public:
	
	
	int magnets; // For sorting through arrays. Could be hardcoded later
	int clicks; // Current number of encoder clicks
	bool calibrationDone; // To check to see if everything got calibrated correctly
	bool override; // If something breaks in the calibration, this jumps to the presets
	bool ascending; // To check what the elevator is doing currently
	bool descending;
	
	
	ElevatorSystem::ElevatorSystem (int elvTalon1, int elvTalon2, int elvPortTop, int elvPortBot, Encoder *encoder, map<string, float> constants){
		elvMotor1 = new Talon(elvTalon1);
		elvMotor2 = new Talon(elvTalon2);
		elvEncoder = encoder; 
		
		topHallSensor = new DigitalInput(elvPortTop);
		bottomHallSensor = new DigitalInput(elvPortBot);
		
		// TODO do these start triggered??
		topTriggered = false;
		bottomTriggered = false; 
		calibrationDone = false;
		ascending = false;
		descending = false;
		
		// The KEY is the HIGH clicks, the VALUE is the LOW clicks
		presetsLow 	   = new map<string, float>[5];
		calibratedLow  = new map<string, float>[5];
		presetsHigh    = new map<string, float>[5];
		calibratedHigh = new map<string, float>[5];
		correctLow     = new map<string, float>[5];
		correctHigh    = new map<string, float>[5];
		
		kp = constants["ELV_KP"];
		ki = constants["ELV_KI"];
		kd = constants["ELV_KD"];
		elvPID = new ControlLoop(kp, ki, kd);
		
	}
	

	ElevatorSystem::~ElevatorSystem () { /*Deconstructor*/ }
	
	
	void UseCalibrated () {
		correctLow = calibratedLow;
		correctHigh = calibratedHigh;
	}
	
	void UsePreset () {
		correctLow = presetsLow;
		correctHigh = presetsHigh;
	}
	
	void MotorsSet (float speed) {
		elvMotor1->Set(speed);
		elvMotor2->Set(speed);
	}
	
	void CalibrateHalls () {
		
		// For looping through 
		magnets = 0;
		// None triggered to start
		highFilled = false; 
		lowFilled = false;
		
		float high = 0;
		float low = 0;
		
		// For all four magnets
		while (magnets < 5) {
			
			// Know always where the magnets are. TODO how to make this more precise
			clicks = elvEncoder->Get();
			
			// The top sensor is triggered
			// Now determine direction of movement
			if(TopHallTriggered()){
				// Going up
				if (bottomTriggered){
					high = clicks;
					
					//Reset flags
					topTriggered = false;
					bottomTriggered = false;
					highFilled = true;
				}
				// Going down
				else if (!bottomTriggered){
					low = clicks;
					lowFilled = true;
				}
			}
			
			// Now bottom sensor is triggered
			else if (BottomHallTriggered()){
				// Going down
				if (topTriggered) {
					low = clicks;
					
					// Reset flags
					topTriggered = false; 
					bottomTriggered = false;
					lowFilled = true;
				}
				// Going up
				else if (!topTriggered) {
					high = clicks;
					highFilled = true;
				}
			}
			
			// Signals that we need to move to the next magnet
			if (highFilled && lowFilled){
				magnets++;
				highFilled = false;
				lowFilled = false;
			}
			
			calibratedLow[magnets]["LOW"] = low;
			calibratedHigh[magnets]["HIGH"] = high;
			
			// If something breaks
			if (override)
			break;
			
		}
		
		// Established whether we need the presets or not
		if (!override){
			calibrationDone = true;
		}
		else if (override) {
			calibrationDone = false;
		}
	}
	
	
	void SetElevatorSpeed (float speed) {
		
		// As to not burn out the motors
		//if (!AtBottom() && speed < 0) {
		if (!BottomHallTriggered() && speed < 0) {	
			MotorsSet(speed);
		}
		else {
			MotorsSet(0.0);
		}
		
		//if (!AtTop() && speed > 0) {
		if (!TopHallTriggered() && speed < 0) {
			MotorsSet(speed);
		}
		else {
			MotorsSet(0.0);
		}
		
	}
	
	
	void StopElevator () {
		
		MotorsSet(0.0);
		
		ascending = false;
		descending = false;
	}
	
	
	void RunElevatorDown (int mag) {
		descending = true;
		ascending = false;
		
		float elvSpeed = elvPID->RunLoop(FindDescendingAmount(correctLow[mag], correctHigh[mag], false));
		
		MotorsSet(0.5*elvSpeed);
	}
	
	
	void RunElevatorUp (int mag) {
		ascending = true;
		descending = false;
		
		float elvSpeed = elvPID->RunLoop(FindAscendingAmount(correctLow[mag], correctHigh[mag], false));
			
		MotorsSet(0.5*elvSpeed);
		
	}
	
	
	// Going up
	float FindAscendingAmount (map<string, float> clicksLow, map<string, float> clicksHigh, bool top ) {
				
		float difference;
		if (top) {
			difference = clicksHigh["HIGH"] - elvEncoder->Get();
		}
		else if (!top) {
			difference = clicksLow["LOW"]  - elvEncoder->Get();
		}
		
		return difference;
	}
	
	
	// Going down
	float FindDescendingAmount (map<string, float> clicksLow, map<string, float> clicksHigh, bool top ) {
		
		float difference; 
		if (top) {
			difference = elvEncoder->Get() - clicksHigh["HIGH"];
		}
		else if (!top) {
			difference = elvEncoder->Get() - clicksLow["LOW"];
		}
		return difference;
	}
	
	

	
	bool TopHallTriggered () {
		if (topHallSensor->Get() == 1) {
			topTriggered = true;
			return true;
		}
		return false;
	}
	
	
	bool BottomHallTriggered () {	
		if (bottomHallSensor->Get() == 1) {
			bottomTriggered = true;
			return true;
		}
		return false;
	}
	
	
	bool AtBottom () {
		if(elvEncoder->Get() <= correctLow[0]["LOW"]){
			return true;
		}
		else {
			return false;
		}
	}
	
	
	bool AtTop () {
		if(elvEncoder->Get() >= correctHigh[4]["HIGH"]) {
			return true;
		}
		else {
			return false;
		}
	}
	
	
	float CurrentSpot () {
		return elvEncoder->Get();
	}
	
	
};


#endif
