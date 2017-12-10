#ifndef MAG_CALIB_H
#define MAG_CALIB_H

class MagCalib
{
public:
	// call this first with multiple values, spread evenly across one or more full circles.
	void putCenterCalibValue(float x, float y);

	// call this before calling putCalibValue()
	void finishCenterCalib();

	void putCalibValue(float fDegree, float x, float y);

	// returns the offset in degrees for the ".0" position
	float finishCalib();

private:
	float m_fCenterX = 0.0f;
	float m_fCenterY = 0.0f;
	int m_iCenterCount = 0;

	float m_fHeading0X = 0.0f;
	float m_fHeading0Y = 0.0f;

};


#endif


