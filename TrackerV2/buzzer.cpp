#include "buzzer.h"

Buzzer::Buzzer(uint8_t pin)
	: m_pin(pin)
{
	pinMode(pin, OUTPUT);
}

void Buzzer::setHertz(unsigned int frequency)
{
	this->m_frequency = frequency;
}

void Buzzer::setBuzzing(uint32_t msBuzz, uint32_t msSilence, int repeats)
{
	this->m_msBuzz = msBuzz;
	this->m_msSilence = msSilence;
	this->m_iNumRemainingRepeats = repeats;

	this->m_msNext = millis();
}

void Buzzer::setSilent()
{
	this->setBuzzing(0, 0);
}

void Buzzer::update()
{
	auto now = millis();

	if (now > this->m_msNext)
	{
		bool bBuzz = ((this->m_iNumRemainingRepeats == -1 || this->m_iNumRemainingRepeats > 0) && this->m_msBuzz > 0);

		if (this->m_bBuzzing)
		{
			noTone(this->m_pin);
			if (bBuzz)
			{
				this->m_msNext = now + this->m_msSilence;
				if (this->m_iNumRemainingRepeats > 0)
					this->m_iNumRemainingRepeats--;
			}
			this->m_bBuzzing = false;
		}
		else if (bBuzz)
		{
			tone(this->m_pin, this->m_frequency);
			this->m_msNext = now + this->m_msBuzz;
			
			this->m_bBuzzing = true;
		}
	}
}