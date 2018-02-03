#include "toggle_output.h"
#include <Arduino.h>

ToggleOutput::ToggleOutput(uint8_t pin)
	: m_pin(pin)
{
	pinMode(this->m_pin, OUTPUT);
	digitalWrite(this->m_pin, LOW);
}

void ToggleOutput::setState(bool bState)
{
	this->m_state = bState;
	digitalWrite(this->m_pin, bState ? HIGH : LOW);
	this->m_msLastAction = millis();
}

void ToggleOutput::set(uint32_t msOn, uint32_t msOff)
{
	if (msOn != this->m_msOn && msOff != this->m_msOff)
	{
		this->setState(msOn > 0);

		this->m_msOn = msOn;
		this->m_msOff = msOff;
	}
}

void ToggleOutput::clear()
{
	this->set(0, 0);
}

void ToggleOutput::update()
{
	if (this->m_msOn > 0 && this->m_msOff > 0)
	{
		auto msNext = this->m_msLastAction +
			(this->m_state ? this->m_msOn : this->m_msOff);
		if (millis() > msNext)
		{
			this->setState(!this->m_state);
		}
	}
}
