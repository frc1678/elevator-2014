#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"

#ifndef COEFFICIENTS_H
#define COEFFICIENTS_H

//TODO Change for elevator AND Gyro
#define kpError 0.05
#define kiError 0.005
#define kdError 0.0

class Coefficients {
	
public:
	float elvErr;
	float elvDeriv;
	float elvInt;
	NetworkTable *elvTable;
	
	
	float gyroErr;
	float gyroDeriv;
	float gyroInt;
	float turn;
	NetworkTable *gyroTable;
	
	
	
	Coefficients::Coefficients () {
	
		elvErr = 0;
		elvDeriv = 0;
		elvInt = 0;
		// for the gyro
		turn = 90;
		
	}
	
	Coefficients::~Coefficients () {
		
	}
	
	
	void SetupTables () {
		elvTable = NetworkTable::GetTable("elvTable");
		gyroTable = NetworkTable::GetTable("gyroTable");
		
		elvTable->PutNumber("kpError", kpError);
		elvTable->PutNumber("kiError", kiError);
		elvTable->PutNumber("kdError", kdError);
		
		
		
	}
	
	void GetValues () {
		elvTable->PutBoolean("Enabled", false);
	
		
	}
	
	void UpdateTables () {
		elvErr = elvTable->GetNumber("kpError", kpError);
		elvInt = elvTable->GetNumber("kiError", kiError);
	}
	
	
	
	
	
	
	
	
	
	
};




#endif
