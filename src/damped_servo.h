#ifndef DAMPED_SERVO_H
#define DAMPED_SERVO_H

#include <Arduino.h>
#include <Filters.h>
#include <Servo.h>

/**
 * Damps a servo's position by applying a LPF
 */
class DampedServo
{
public:
	DampedServo(uint8_t pin, int min_pwm, int max_pwm);

public:
	void setPosition(float pos);

	void update();

private:
	const int m_min_pwm;
	const int m_max_pwm;

	float m_pos = 0.f;
	Servo m_servo;
	FilterOnePole m_lpf;
};



#endif


