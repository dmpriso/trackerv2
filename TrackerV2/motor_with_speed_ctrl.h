#ifndef MOTOR_WITH_SPEED_CTRL_H
#define MOTOR_WITH_SPEED_CTRL_H

#include "motor.h"
#include <PID_v1.h>


class MotorWithSpeedCtrl
{
public:
	MotorWithSpeedCtrl(Motor& motor, 
		double p, double i, double d);

public:
	void setSpeed(double stepsPerSecond);
	
	// to be called from a timer every 1000us
	void on10000usElapsed();

	// to be called from main loop
	void update();

	int position();

private:
	Motor& motor;
	PID pid;

	double measured_speed = 0.;
	double output = 0.;
	double desired_speed = 0.;

	int last_pos = 0;
};

#endif


