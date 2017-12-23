#include "rotor_with_pos_ctrl.h"
#include <math.h>

double normalizeAngle360d(double val) {
	val = fmod(val, 360.);
	if (val < 0.) val += 360.;
	return val;
}


RotorWithPosCtrl::RotorWithPosCtrl(MotorWithSpeedCtrl& motor,
	double p, double i, double d,
	double steps_per_revolution,
	PositionCallback cb)
	: motor(motor),
	pid(&measured_pos, &output, &desired_pos, p, i, d, DIRECT),
	cb(cb),
steps_per_revolution(steps_per_revolution)
{
	// limit to 120deg/sec
	pid.SetOutputLimits(-120.f, 120.f);
	pid.SetMode(AUTOMATIC);
	pid.SetSampleTime(10);
	this->measured_pos = this->desired_pos = this->stepToDeg((double)motor.position());
}

void RotorWithPosCtrl::setPosition(double positionDegrees)
{
	this->bPosCtrlActive = true;

	positionDegrees = normalizeAngle360d(positionDegrees);

	// we need to find out the nearest actual desired position
	// our positions may already have accumulated many full rotations
	auto add = this->measured_pos - normalizeAngle360d(this->measured_pos);
	positionDegrees += add;

	auto diff1 = abs(positionDegrees - this->measured_pos);
	
	// try the other way around
	auto tmp = positionDegrees > this->measured_pos ? -360. : 360.;
	if (abs(positionDegrees + tmp - this->measured_pos) < diff1)
	{
		positionDegrees += tmp;
	}
	
	this->desired_pos = positionDegrees;
}

void RotorWithPosCtrl::setContinuousSpeed(double degPerSec)
{
	this->bPosCtrlActive = false;
	this->motor.setSpeed(this->degToStep(degPerSec));
}


void RotorWithPosCtrl::on10000usElapsed()
{
	this->motor.on10000usElapsed();
}

void RotorWithPosCtrl::update()
{
	this->measured_pos = this->stepToDeg(this->motor.position());
	auto pos360 = normalizeAngle360d(this->measured_pos);
	if (pos360 != this->last_reported_pos)
	{
		this->last_reported_pos = pos360;
		if (nullptr != this->cb)
			this->cb(pos360);
	}

		//Serial.print(measured_pos);
		//Serial.print("!=");
		//Serial.print(desired_pos);
		//Serial.print("=>");
		//Serial.println(output);

	if (this->bPosCtrlActive)
	{
		if (this->pid.Compute())
		{
			this->motor.setSpeed(this->degToStep(this->output));
		}
	}
	this->motor.update();
}
