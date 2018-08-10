#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

class Buzzer
{
public:
	Buzzer(uint8_t pin);

public:
	void setHertz(unsigned int frequency);
	void setBuzzing(uint32_t msBuzz, uint32_t msSilence, int repeats = -1);
	void setSilent();

	void update();

private:
	const uint8_t m_pin;

	unsigned int m_frequency = 1000;

	int m_iNumRemainingRepeats = 0;
	bool m_bBuzzing = false;
	uint32_t m_msBuzz = 0;
	uint32_t m_msSilence = 0;
	uint32_t m_msNext = 0;
};


#endif


