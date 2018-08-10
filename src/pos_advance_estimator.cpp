#include "pos_advance_estimator.h"

PosAdvanceEstimator::PosAdvanceEstimator(int iMillisAdvance)
	: m_iMillisAdvance(iMillisAdvance),
	m_lpf(LOWPASS, .2f, 0.f)
{}

void PosAdvanceEstimator::cleanup()
{
	int iNumRemove = 0;
	auto now = millis();

	for (int i = 0; i < min(this->m_iNumLastPositions, this->m_lastPositionsSize); i++)
	{
		if ((now - this->m_lastPositions[i].ts) < 5000)
			break;

		++iNumRemove;
	}
	if (iNumRemove > 0)
		for (int i = 0; i < this->m_iNumLastPositions - iNumRemove; i++)
		{
			this->m_lastPositions[i] = this->m_lastPositions[i + iNumRemove];
		}
	this->m_iNumLastPositions -= iNumRemove;
}


bool PosAdvanceEstimator::calcSpeed(float dLat, float dLon, float dAlt, float dTime, float& speed)
{
	if (dTime > 1.f)
	{
		// calc number of meters covered above earth (as seen from above)
		float numDeg = sqrtf(powf(dLat, 2.f) + powf(dLon, 2.f));
		float numMeters = numDeg * 111139.f;

		// calc total number of meters covered, also including altitude
		numMeters = sqrtf(powf(numMeters, 2.f) + powf(dAlt, 2.f));

		// speed in m/s
		speed = 1000.f * numMeters / dTime;

		return true;
	}
	else return false;
}

void PosAdvanceEstimator::putSpeed(float speedMs)
{
	this->m_lpf.input(speedMs);
}

float PosAdvanceEstimator::getSpeed()
{
	return this->m_lpf.output();
}

PosCalc::Position PosAdvanceEstimator::calc(const PosCalc::Position& pos)
{
	this->cleanup();

	bool bNew = 0 == this->m_iNumLastPositions || this->m_lastPositions[this->m_iNumLastPositions - 1].pos != pos;
	if (bNew)
	{
		if (this->m_iNumLastPositions == this->m_lastPositionsSize)
		{
			//Serial.print(this->m_lastPositionsSize);
			// make some space
			for (int i = 0; i < this->m_iNumLastPositions - 1; i++)
			{
				this->m_lastPositions[i] = this->m_lastPositions[i + 1];
			}
			--this->m_iNumLastPositions;
		}
		this->m_lastPositions[this->m_iNumLastPositions++] = PosWithTime(millis(), pos);

		//Serial.println("ADVEST new pos. Items:");
		for (int i = 0; i < this->m_iNumLastPositions; i++)
		{
			//Serial.print(i);
			//Serial.print("#: ");
			//auto& pos = this->m_lastPositions[i];
			//Serial.print(pos.pos.lat, 8);
			//Serial.print(" ");
			//Serial.print(pos.pos.lon, 8);
			//Serial.print(" ");
			//Serial.print(pos.pos.alt, 2);
			//Serial.print(" ms=");
			//Serial.println(pos.ts);
		}
		//Serial.println("");
	}
	if (this->m_iNumLastPositions >= 2)
	{
		// calc diff vector
		auto pos1 = this->m_lastPositions[0];
		auto pos2 = this->m_lastPositions[this->m_iNumLastPositions - 1];

		auto dLat = pos2.pos.lat - pos1.pos.lat;
		auto dLon = pos2.pos.lon - pos1.pos.lon;
		auto dAlt = pos2.pos.alt - pos1.pos.alt;
		auto dTime = (float)pos2.ts - (float)pos1.ts;

		// IMPORTANT NOTE
		// First of all, in order to keep things simple & stable, we want to calculate position advance estimation solely on
		// GPS positions and not rely on mavlink data like heading and speed, which may be incorrect or unavailable, especially
		// with mavlink emulation on non-mavlink autopilots.
		// However, telemetry data may arrive in "bursts" which leads to the wrong impression that a distance has been covered
		// in extremely short time, causing the estimator to overreact as if the aircraft would travel with mach 1 or so.
		// Therefore, we do a speed calculation here and put a low pass filter over it.

		float momentarySpeedMs = 0.f;
		bool bSpeedCalculated = this->calcSpeed(dLat, dLon, dAlt, dTime, momentarySpeedMs);
		//Serial.print(momentarySpeedMs);
		//Serial.print(" ");
		//Serial.println(bSpeedCalculated);
		if (bSpeedCalculated && momentarySpeedMs > .1f)
		{
			if (bNew)
			{
				// recalculate speed when new data has arrived.
				this->putSpeed(momentarySpeedMs);
			}

			//Serial.println("ADVEST data:");
			//Serial.print("pos1 lat=");
			//Serial.print(pos1.pos.lat, 8);
			//Serial.print(" lon=");
			//Serial.print(pos1.pos.lon, 8);
			//Serial.print(" time=");
			//Serial.println(pos1.ts);
			//Serial.print("pos2 lat=");
			//Serial.print(pos2.pos.lat, 8);
			//Serial.print(" lon=");
			//Serial.print(pos2.pos.lon, 8);
			//Serial.print(" time=");
			//Serial.println(pos2.ts);

			// get speed
			auto filteredSpeedMs = this->getSpeed();

			// calc factor based on momentary speed as opposed to filtered speed
			auto dTime2 = (float)(millis() + this->m_iMillisAdvance) - (float)pos2.ts;
			auto dFactor = (dTime2 / 1000.f) * filteredSpeedMs / momentarySpeedMs;

			//Serial.print("millis=");
			//Serial.print(millis());
			//Serial.print(" momentarySpeedMS=");
			//Serial.print(momentarySpeedMs);
			//Serial.print(" filteredSpeedMs=");
			//Serial.print(filteredSpeedMs);
			//Serial.print(" dFactor=");
			//Serial.println(dFactor);
			//Serial.println("");

			// add up to estimated position
			auto rLat = pos2.pos.lat + dLat * dFactor;
			auto rLon = pos2.pos.lon + dLon * dFactor;
			auto rAlt = pos2.pos.alt + dAlt * dFactor;

			// return that one
			return PosCalc::Position(rLat, rLon, rAlt);
		}
	}
	// no estimation possible: simply return reported position
	return pos;
}
