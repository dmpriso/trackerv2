#include "motor_with_speed_ctrl.h"

MotorWithSpeedCtrl::MotorWithSpeedCtrl(Motor& motor,
	double p, double i, double d)
	: motor(motor),
	pid(&measured_speed, &output, &desired_speed, p, i, d, DIRECT)
{
	pid.SetOutputLimits(-1000., 1000.);
	pid.SetMode(AUTOMATIC);
	pid.SetSampleTime(10);
	this->last_pos = motor.read();
}

void MotorWithSpeedCtrl::setSpeed(double stepsPerSecond)
{
	this->desired_speed = stepsPerSecond;
}

void MotorWithSpeedCtrl::on10000usElapsed()
{
	int pos = motor.read();
	int diff = pos - this->last_pos;
	this->last_pos = pos;
	this->measured_speed = (double)diff * 100.;
}

void MotorWithSpeedCtrl::update()
{
	//Serial.print(this->measured_speed);
	//Serial.print(" ");
	//Serial.print(this->desired_speed);
	//Serial.print(" ");
	//Serial.print(this->output);
	if (this->pid.Compute())
	{
		this->motor.setSpeed((int)output);
	}
	motor.update();
	//Serial.println();
}

int MotorWithSpeedCtrl::position()
{
	return this->motor.read();
}