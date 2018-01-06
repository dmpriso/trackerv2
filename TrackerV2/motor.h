#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Encoder.h>
#include <Filters.h>

class Motor
{
public:
	Motor(uint8_t pwm_pin,
		uint8_t dir_pin,
		uint8_t enc_pin_1,
		uint8_t enc_pin_2,
		bool reverse = false);

public:
	/**
	 * @param speedPermille Speed in permille. Can be positive or negative.
	 */
	void setSpeed(int speedPermille);
	int read();

	void update();

private:
	void internalSetSpeed(float speedPermille);

private:
	Encoder m_enc;
	const uint8_t m_pwm_pin;
	const uint8_t m_dir_pin;
	const bool m_reverse;
	FilterOnePole lpf;
	float f_input = 0.f;
	

};

#endif
