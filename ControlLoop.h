#include "WPILib.h"
#include "Coefficients.h"


#ifndef CONTROLLOOP_H
#define CONTROLLOOP_H



class ControlLoop {

	Timer *PIDTimer;

	float integral;
	float lastErr;
	
	float kp;
	float ki;
	float kd;
	
	
	
public:
	
	ControlLoop::ControlLoop (float inKP, float inKI, float inKD) {
		
		kp = inKP;
		ki = inKI;
		kd = inKD;
		
	}
	
	ControlLoop::~ControlLoop () { /* Deconstructor*/ }
	
	void StartLoop () {
		PIDTimer->Reset();
		PIDTimer->Start();
			
	}
	
	void ResetLoop () {
		PIDTimer->Reset();
	}
	
	void Stop() {
		PIDTimer->Stop();
	}

	
	
	// Call this once every loop
	float CalibrateLoop (float err) {
		
		// Find the difference between where you want to end and where you are
		// err = fabs(target - input);
		// Calculate how much time has past
		float dt = PIDTimer->Get();
		// Reset the time to zero
		PIDTimer->Reset();
		// Calculate the integral 
		integral += err * dt;
		// Calculate the derivative (not in use)
		float der = (err + lastErr)/dt;
		// Reset the error
		lastErr = err;
		
		// PID coefficients for driving straight, but NOT turning
		//MV output
		return kp*err + ki*integral + kd*der;
		//return 0.05*err + 0.005*integral + 0.00*der;
	}
	
	// err is the difference between the current state and the ending state
	float RunLoop (float err) {
		
		
		float output = CalibrateLoop(err);
		
		
		return output + 1;  // Add 1 so it's proportional
		
	}

};
	
		//if the angle is positive (turning to the left)
	//		if (GetFilterAngle() > 0){
	//			// subtract the range to find how off the angle is from 0
	//			difference = GetFilterAngle(); 
	//			
	//		}
	//		// the angle is negative (turning to the right)
	//		else {
	//			// add the range because the angle is negative
	//			// we want this value to be positive 
	//			difference = fabs(GetFilterAngle());
	//		}
	//		
	//		difference = (sqrt(difference))/360;
	//		
	//		float percent = 1 + (difference*25);
	//		
	//		return percent;
	
	
	//	float TurningPID (float target) {
	//		
	//		//PV input
	//		float input = GetFilterAngle();
	//		
	//		// Find the difference between where you want to end and where you are
	//		float err = fabs(target - input);
	//		// Calculate how much time has past
	//		float dt = PIDTimer->Get();
	//		// Reset the time to zero
	//		PIDTurnTimer->Reset();
	//		// Calculate the integral 
	//		turnInt += err * dt;
	//		// Calculate the derivative (not in use)
	//		float der = (err + lastErr)/dt;
	//		// Reset the error
	//		turnErr = err;
	//		
	//		// PID coefficients for driving straight, but NOT turning
	//		//MV output
	//		float MV = 0.01*err + 0.00*turnInt + 0.00*der;
	//		
	//		return MV + 1;
	//	}

	













#endif 
