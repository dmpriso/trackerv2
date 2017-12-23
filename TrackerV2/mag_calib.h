#ifndef MAG_CALIB_H
#define MAG_CALIB_H

class MagCalib
{
private:
	struct CalibEntry
	{
		float x = 0;
		float y = 0;
		int cnt = 0;
	};

public:
	void putCalibValue(float fDegree, float x, float y);

	// returns the offset in degrees for the ".0" position
	float finishCalib();

private:
	CalibEntry entries[360];
	int m_iNumEntries;
};


#endif


