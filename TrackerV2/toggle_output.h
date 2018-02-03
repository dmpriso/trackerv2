#pragma once

#include <Arduino.h>

class ToggleOutput
{
public:
	ToggleOutput(uint8_t pin);

public:
	void set(uint32_t msOn, uint32_t msOff);
	void clear();

	void update();

private:
	void setState(bool bState);

private:
	const uint8_t m_pin;

	uint32_t m_msOn = 0;
	uint32_t m_msOff = 0;
	bool m_state = false;
	uint32_t m_msLastAction = 0;
};