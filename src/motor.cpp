#include "motor.h"

Motor::Motor(uint8_t pwm_pin,
	uint8_t dir_pin,
	uint8_t enc_pin_1,
	uint8_t enc_pin_2, 
	bool reverse)
	: m_enc(enc_pin_1, enc_pin_2),
	m_pwm_pin(pwm_pin),
	m_dir_pin(dir_pin),
	m_reverse(reverse),
	lpf(LOWPASS, 0.3)
{
	pinMode(dir_pin, OUTPUT);
	digitalWrite(dir_pin, LOW);
	pinMode(pwm_pin, OUTPUT);
	analogWriteResolution(10);
}

void Motor::internalSetSpeed(float speedPermille)
{
	auto dir = (((speedPermille >= 0.f) ^ this->m_reverse) ? LOW : HIGH);
	digitalWrite(this->m_dir_pin, dir);

	//Serial.print("Speed: ");
	//Serial.print(speedPermille);
	//Serial.print(" Dir: ");
	//Serial.println(dir);
	analogWrite(this->m_pwm_pin, (int)(min(abs(speedPermille) * 1024.f / 1000.f, 1023.f)));
}

void Motor::setSpeed(int speedPermille)
{
	this->f_input = (float)speedPermille;
}

void Motor::update()
{
	lpf.input(this->f_input);

	this->internalSetSpeed(lpf.output());
}

int Motor::read()
{
	return this->m_enc.read();
}
