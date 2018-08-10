#ifndef ROTOR_WITH_POS_CTRL_H
#define ROTOR_WITH_POS_CTRL_H

#include "motor_with_speed_ctrl.h"

class RotorWithPosCtrl
{
public:
	typedef void(*PositionCallback)(double pos);

public:
	RotorWithPosCtrl(MotorWithSpeedCtrl& motor,
		double p, double i, double d,
		double steps_per_revolution,
		PositionCallback cb);
public:
	void setPosition(double positionDegrees);
	void setContinuousSpeed(double degPerSec);

	// to be called from a timer every 1000us
	void on10000usElapsed();

	// to be called from main loop
	void update();

private:
	inline double degToStep(float deg) 
	{ 
		return deg * steps_per_revolution / 360.; 
	}
	inline double stepToDeg(float step) 
	{ 
		auto tmp = step * 360.f / steps_per_revolution; 
		if (this->bPosCtrlActive)
			return round(tmp);
		else
			return tmp;
	}

private:
	MotorWithSpeedCtrl& motor;
	PID pid;
	bool bPosCtrlActive = false;

	double measured_pos = 0.;
	double output = 0.;
	double desired_pos = 0.;

	double last_reported_pos = 0.;
	const PositionCallback cb;

	const double steps_per_revolution;
};

#endif

