#include "mag_calib.h"
#include "tools.h"

#include <Arduino.h>
#include <math.h>

void MagCalib::putCenterCalibValue(float x, float y)
{
	this->m_fCenterX += x;
	this->m_fCenterY += y;
	++this->m_iCenterCount;
}

void MagCalib::finishCenterCalib()
{
	this->m_fCenterX /= (float)this->m_iCenterCount;
	this->m_fCenterY /= (float)this->m_iCenterCount;
}

void MagCalib::putCalibValue(float fDegree, float x, float y)
{
	// center value
	x -= this->m_fCenterX;
	y -= this->m_fCenterY;

	// calc angle
	float heading = rad2deg(atan2(y, x));

	// calc heading of "0" position
	float heading0 = normalizeAngle360(heading - fDegree);
	float heading0Rad = deg2rad(heading0);

	//Serial.print(F("Calib Heading: "));
	//Serial.print(heading);
	//Serial.print(F(" -> "));
	//Serial.print(heading0);
	//Serial.print(F(" "));
	//Serial.println(heading0Rad);

	// add to sum vector
	this->m_fHeading0X += cos(heading0Rad);
	this->m_fHeading0Y += sin(heading0Rad);
}

float MagCalib::finishCalib()
{
	float fHeading0Rad = atan2(this->m_fHeading0Y, this->m_fHeading0X);
	float fHeading0 = rad2deg(fHeading0Rad);

	//Serial.print(F("Calib FINAL Heading0: "));
	//Serial.print(fHeading0);
	//Serial.print(F(" "));
	//Serial.println(fHeading0Rad);

	return fHeading0;
}

