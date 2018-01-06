#ifndef POS_ADVANCE_ESTIMATOR_H
#define POS_ADVANCE_ESTIMATOR_H

#include "pos_calc.h"

/**
* Predicts current/future position based on past position
*/
class PosAdvanceEstimator
{
private:
	struct PosWithTime
	{
		inline PosWithTime(uint32_t ts, const PosCalc::Position& pos)
			: ts(ts), pos(pos)
		{}

		PosWithTime() = default;
		PosWithTime(const PosWithTime&) = default;
		PosWithTime& operator=(const PosWithTime&) = default;

		uint32_t ts;
		PosCalc::Position pos;
	};

public:
	/**
	* @param iMillisAdvance May be set to positive value in order to compensate for telemetry delay
	*/
	PosAdvanceEstimator(int iMillisAdvance = 0);

public:
	PosCalc::Position calc(const PosCalc::Position& pos);

private:
	void cleanup();

private:
	const int m_iMillisAdvance;

	PosWithTime m_lastPositions[2];
	int m_iNumLastPositions = 0;
	const int m_lastPositionsSize = sizeof(m_lastPositions) / sizeof(PosWithTime);

};

#endif


