/* Class for the MPU6050 non-KOP gyro
 * Plugs into the I2C port on digital sidecar
 * Relevant datasheets:
 * http://www.invensense.com/mems/gyro/documents/RM-MPU-6000A-00v4.2.pdf (register map)
 * http://invensense.com/mems/gyro/documents/PS-MPU-6000A-00v3.4.pdf (datasheet)
 */
//TODO fix magic numbers

#include "WPILib.h"
#include "ControlLoop.h"
#include "math.h"
//#include "CrioFile.h"

#ifndef MPU6050_I2C_H
#define MPU6050_I2C_H

#define PI 3.14159265358979323846
#define XACCEL 0
#define YACCEL 1
#define ZACCEL 2

#define baseWidth 24.25
#define wheelCir 12.5663706

class MPU6050_I2C
{
	//variables
	DigitalModule *dModule;
	I2C *me;
	
	ControlLoop *VOLoop;
	ControlLoop *OutLoop;
	ControlLoop *InLoop;
	
	float gyroAngle;
	
	float gyroTotal;
	float gyroOut;

	float filteredAngle;
	
	float gyroOffset;
	float XOffset;
	float YOffset;
	
	float VO_kp, VO_ki, VO_kd, turnAngle, turnRadius;
	float OUT_kp, OUT_ki, OUT_kd, IN_kp, IN_ki, IN_kd;
	
	timespec lastTime;
	
//	Timer myNewTimer;

	
public:
	MPU6050_I2C (map<string, float> constants, map<string, float> autoConsts)
	{
		// Accessing the digital module that the gyro is connected to 
		dModule = DigitalModule::GetInstance(1);
		
		// 0x68 is the address of the gyro
		me = dModule->GetI2C(0x68<<1); 
		
		// Creating the new timers
		
		VO_kp = constants["GYRO_KP"];
		VO_ki = constants["GYRO_KI"];
		VO_kd = constants["GYRO_KD"];
		turnAngle = autoConsts["GYRO_ANGLE"];
		turnRadius = autoConsts["GYRO_TURNRAD"];
		
		VOLoop = new ControlLoop(VO_kp, VO_ki, VO_kd);
		OutLoop = new ControlLoop(OUT_kp, OUT_ki, OUT_kd);
		InLoop = new ControlLoop(IN_kp, IN_ki, IN_kd);
		
		// Initate the gyro so everything is reset
		// And the offsets can be calculated 
		InitGyro();
		

		
//		timespec ts;
//		clock_gettime(CLOCK_MONOTONIC, &ts);
		
	}
	
	
	void InitGyro ()
	{
		
		// Set this to 0 to be updated each loop
		filteredAngle = 0;
		
		// Find the offsets for the gyro and accelerometer
		gyroOffset = FindGyroOffset();
		XOffset = FindAccelOffset(XACCEL);
		YOffset	= FindAccelOffset(YACCEL);
		
		// Set the angles to zero
		gyroAngle = 0.0;
		
		// Start the timers
		clock_gettime(CLOCK_MONOTONIC, &lastTime);

		// Write the this address on the gyro to take it out of sleep mode
		me->Write(0x6B, 0); //PWR_MGMT_1:=0 take me out of sleep mode
	}
	
	float FindGyroOffset ()
	{
		// Loop through a number of times (20)
		for (int i = 0; i < 20; i++)
		{
			// Add the values together
			gyroOffset += GyroRead();
		}
		
		// Divide to find the avergae for the offset
		gyroOffset /= 20;
		
		// Return the offset
		return gyroOffset;
		
	}
	
	
	// Take in a value corresponding to an axis
	// 0 is X axis, 1 is Y axis, 2 is Z axis
	float FindAccelOffset (int axis)
	{
		float accOffset;
		
		// Loop through a number of times (20)
		for (int i = 0; i < 20; i++)
		{
			// Add the values together 
			accOffset += AccelRead(axis);
		}
		
		// Divide to find the average for the offset
		accOffset /= 20;
		
		// Return that offset
		return accOffset;
	}
	
	float GetXOffset () {
		return XOffset;
	}
	
	float GetYOffset () {
		return YOffset;
	}

	
	// Set everything back to zero and restart the timers
	void Reset ()
	{
		gyroAngle = 0.0;
		gyroTotal = 0.0;
		gyroOut = 0.0;
		filteredAngle = 0.0;
		gyroOffset = 0.0;
		XOffset = 0.0;
		YOffset = 0.0;
//		lastTime = {0.0, 0.0};
		
//		gyro>Reset();
	}

	
	float FindVO () {
		// SHould be with gyros
		float currentVal = GetGyroAngle();
		float endVal = turnAngle;
		float VO = VOLoop->CalibrateLoop(currentVal - endVal);
		
		return VO * (2 * PI * turnRadius / 360.0);
	}
	
	float FindVI (float VO) {
		return VO * ((turnRadius - (baseWidth/2)) / (turnRadius + (baseWidth/2)));
	}
	
	float FindOutsideSpeed (Encoder *outsideEncoder, float VO) {
		float currentVal = outsideEncoder->GetRate() / 360.0 * wheelCir;
		float endVal = VO;
		float driveSpeed = OutLoop->CalibrateLoop(currentVal - endVal);
		
		return driveSpeed;
	}
	
	float FindInsideSpeed (Encoder *insideEncoder, float VI) {
		float currentVal = insideEncoder->GetRate() / 360.0 * wheelCir;
		float endVal = VI;
		float driveSpeed = InLoop->CalibrateLoop(currentVal - endVal);
		
		return driveSpeed;
	}
	
	
	void LeftSwoopTurn (Encoder *rightEncoder, Encoder *leftEncoder, RobotDrive *drivetrain) {
		float VO = FindVO();
		if(VO > 45) {
			VO = 45;
		}
		float VI = FindVI(VO);
		
		
		
		float rightSpeed = FindOutsideSpeed(rightEncoder, VO);
		float leftSpeed  = FindInsideSpeed(leftEncoder, VI);
		
		
		
		drivetrain->TankDrive(leftSpeed, rightSpeed);
		
	}
	
	
	void RightSwoopTurn (Encoder *rightEncoder, Encoder *leftEncoder, RobotDrive *drivetrain) {
		float VO = FindVO();
		if(VO > 45) {
			VO = 45;
		}
		float VI = FindVI(VO);
		
		float rightSpeed = FindInsideSpeed(rightEncoder, VI);
		float leftSpeed  = FindOutsideSpeed(leftEncoder, VO);
		
		drivetrain->TankDrive(leftSpeed, rightSpeed);
		
	}
	
	
	void RightPointTurn (Encoder *rightEncoder, Encoder *leftEncoder, RobotDrive *drivetrain) {
		float VO = FindVO();
		
		float rightSpeed = FindInsideSpeed(rightEncoder, VO);
		float leftSpeed = FindOutsideSpeed(leftEncoder, VO);
		
		drivetrain->TankDrive(-leftSpeed, rightSpeed);
		
	}
	
	void LeftPointTurn (Encoder *rightEncoder, Encoder *leftEncoder, RobotDrive *drivetrain) {
		float VO = FindVO();
		
		float rightSpeed = FindOutsideSpeed(rightEncoder, VO);
		float leftSpeed = FindInsideSpeed(leftEncoder, VO);
		
		drivetrain->TankDrive(leftSpeed, -rightSpeed);
		
	}
	
	
	// Reads raw values from the gyro
	// Converts them to be meaningful
	float GyroRead ()
	{
		uint8_t gyroValue = 7;
		
		int GYRO_ZOUT_H;
		int GYRO_ZOUT_L;
		
	
		// Reading the gyro's turn in the z-axis
		// Reading the high byte from the correct address
		me->Read(0x47, sizeof(gyroValue), &gyroValue);
		GYRO_ZOUT_H = gyroValue;
		
		// Reading the low byte from the correct address
		me->Read(0x48, sizeof(gyroValue), &gyroValue);
		GYRO_ZOUT_L = gyroValue;
			
		// Add the two bytes, most significant first
		gyroOut = (float)((GYRO_ZOUT_H << 8) + GYRO_ZOUT_L);

		
		// Taking 16 bits of all positive data and converting to positive and negative data
		// AKA turning unsigned to signed
		if (gyroOut > 32768) 
		{
			gyroOut -= 32768*2;
		}
		
		gyroOut /= 131.0;   // Gyro scale. Converts raw rate to degrees per sec

		// Return the value as degrees per second
		return gyroOut; // This is degrees per second
		
	}
	
	
	// Can then be used for any acclerometer axis, X, Y or Z
	float AccelRead (int axis)
	{
		uint8_t accValue = 7;
		
		float OutAcc;

		int ACC_OUT_H;
		int ACC_OUT_L;

		// If it's the X axis
		if (axis == XACCEL) 
		{
			// Read the high byte from the correct address
			me->Read(0x3B, sizeof(accValue), &accValue);
			ACC_OUT_H = accValue;
			
			// Reading the low byte from the correct address
			me->Read(0x3C, sizeof(accValue), &accValue);
			ACC_OUT_L = accValue;
		}
		// If it's the Y axis
		else if (axis == YACCEL)
		{
			// Read the high byte from the correct address
			me->Read(0x3D, sizeof(accValue), &accValue);
			ACC_OUT_H = accValue;
			
			// Reading the low byte form the correct address
			me->Read(0x3E, sizeof(accValue), &accValue);
			ACC_OUT_L = accValue;
			
		}
		// If it's the Z axis
		else if (axis == ZACCEL) 
		{
			// Reading the high byte from the correct address
			me->Read(0x3F, sizeof(accValue), &accValue);
			ACC_OUT_H = accValue;
			
			// Reading the low byte from the correct address
			me->Read(0x40, sizeof(accValue), &accValue);
			ACC_OUT_L = accValue;	

		}
		// If something doesn't work, or an incorrect number is given
		else {
			// Just set it to zero
			ACC_OUT_H = 0;
			ACC_OUT_L = 0;
		}
		
		// Add the two bytes, the most significant one first
		OutAcc = (float)((ACC_OUT_H << 8) + ACC_OUT_L);
		
		
		// Was unsigned before, meaning it didn't deal with negative values
		// This deals allows for negative values to be returned 
		// Equivalent of signed 
		if (OutAcc > 32768) 
		{
			OutAcc -= 32768*2;
		}
	
		OutAcc /= 16384.0; // Accelerometer scale
		
		// Return the value 
		return OutAcc;
	}
	
	
	float GyroRate () //reading the raw-ish data from a port
	{
		float gyroRate;
		
		// Subtract the gyro offset
		// TODO this isn't working for some reason. Fix it
		gyroRate = GyroRead() - gyroOffset;
		
		return gyroRate;
	
	
	}
	
	
	float GetAccel ()
	{
		float rotation;	
		
		// Read from each of the accelerometer axes and then subtract offset
		// TODO this isn't really working right either
		float XAcc = AccelRead(XACCEL) - XOffset;
		float YAcc = AccelRead(YACCEL) - YOffset;
		
		// Rotation in degrees for Yaw axis (z-axis)
		rotation = GetYawRotate(XAcc, YAcc);
		
		return rotation;
	}
	
	
	// For the accelerometer scaling of the angle
	float GetYawRotate (float x, float y) {
		
		float degrees;
		degrees = atan2(fabs(y), fabs(x)) * 180 / PI;
		
		return degrees;
		
	}
	
	
	///
	/// This must be called everytime in a loop
	/// More oftenly called, the more acturate the angle will be
	///
	float GetGyroAngle() 
	{
		float currRate = GyroRate();
//		if (currRate < 0.5 && currRate > -0.5) // ((currRate < 0.5 && currRate > -0.5) || currRate - lastRate > 50) Deadzone for rotation?
//		{
//			currRate = 0.0;
//		}
		
		timespec current;
		clock_gettime(CLOCK_MONOTONIC, &current);
		
		float dt = 1000*(current.tv_sec - lastTime.tv_sec) + (current.tv_nsec - lastTime.tv_nsec)/1000000.0;
		lastTime = current;
		gyroAngle += currRate*(dt/1000.0);
		
		return gyroAngle;
		
	}

	
	// Implementation of the complimentary filter
	// TODO needs to be tuned
	float GetFilterAngle ()
	{
		
		filteredAngle = 0.99*(GetGyroAngle()) + 0.01*(GetAccel());
		
		return filteredAngle;
	}
	
	
	void Stop()
	{

	}
	
	
	
	
	
//	
//	// the error (kp) for driving straight
//	float DriveStraightDifference () {
//		float motorSpeed;
//		
//		float difference = fabs(0.0 - GetFilterAngle());
//		
//		motorSpeed = gyroPID->RunLoop(difference);
//		
//		return motorSpeed;
//	}
//	
//	// The error (kp) for turning
//	float TurningDifference () {
//		
//		float difference = fabs(gyroPID->turn - GetFilterAngle());
//		float motorSpeed = gyroPID->RunLoop(difference);
//		
//		return motorSpeed;
//		
//		
//	}
//	
//	
	
	
	
	
	
	
	
	
	
	
//	
//	// Call this once every loop
//	float CalculatePID (float input, float target) {
//		
//		// Find the difference between where you want to end and where you are
//		float err = fabs(target - input);
//		// Calculate how much time has past
//		float dt = PIDTimer->Get();
//		// Reset the time to zero
//		PIDTimer->Reset();
//		// Calculate the integral 
//		integral += err * dt;
//		// Calculate the derivative (not in use)
//		float der = (err + lastErr)/dt;
//		// Reset the error
//		lastErr = err;
//		
//		// PID coefficients for driving straight, but NOT turning
//		//MV output
//		return 0.05*err + 0.005*integral + 0.00*der;
//	}
//	
//	float RunPID (float target) {
//		
//		//PV input
//		float input = GetFilterAngle();
//		
//		float output = CalculatePID(input, target);
//		
//		//output =+ 1;
//		
//		return output + 1;
//	}	
//		
		
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
	
	
	
	

	
	
	
	
};

#endif
