#include "pos_advance_estimator.h"

PosAdvanceEstimator::PosAdvanceEstimator(int iMillisAdvance)
	: m_iMillisAdvance(iMillisAdvance)
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

PosCalc::Position PosAdvanceEstimator::calc(const PosCalc::Position& pos)
{
	this->cleanup();

	bool bNew = 0 == this->m_iNumLastPositions || this->m_lastPositions[this->m_iNumLastPositions - 1].pos != pos;
	if (bNew)
	{
		if (this->m_iNumLastPositions == this->m_lastPositionsSize)
		{
			// make some space
			for (int i = 0; i < this->m_iNumLastPositions - 1; i++)
			{
				this->m_lastPositions[i] = this->m_lastPositions[i + 1];
			}
			--this->m_iNumLastPositions;
		}
		this->m_lastPositions[this->m_iNumLastPositions++] = PosWithTime(millis(), pos);
	}
	if (this->m_iNumLastPositions >= 2)
	{
		// calc diff vector
		auto pos1 = this->m_lastPositions[this->m_iNumLastPositions - 2];
		auto pos2 = this->m_lastPositions[this->m_iNumLastPositions - 1];

		auto dLat = pos2.pos.lat - pos1.pos.lat;
		auto dLon = pos2.pos.lon - pos1.pos.lon;
		auto dAlt = pos2.pos.alt - pos1.pos.alt;
		auto dTime = (float)pos2.ts - (float)pos1.ts;

		if (dTime > 1.f)
		{
			// calc diff up to current time
			auto dTime2 = (float)(millis() + this->m_iMillisAdvance) - (float)pos2.ts;
			auto dFactor = dTime2 / dTime;

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
