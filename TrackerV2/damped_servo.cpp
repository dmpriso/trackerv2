#include "damped_servo.h"

DampedServo::DampedServo(uint8_t pin, int min_pwm, int max_pwm)
	: m_min_pwm(min_pwm),
	m_max_pwm(max_pwm),
	m_lpf(LOWPASS, .3f, 0.f)
{
	this->m_servo.attach(pin);
}

void DampedServo::setPosition(float pos)
{
	this->m_pos = pos;
}

void DampedServo::update()
{
	this->m_lpf.input(this->m_pos);
	this->m_servo.write(this->m_min_pwm +
		(int)((float)(this->m_max_pwm - this->m_min_pwm) * this->m_lpf.output()));
}
