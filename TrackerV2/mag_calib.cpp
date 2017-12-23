#include "mag_calib.h"
#include "tools.h"

#include <Arduino.h>
#include <math.h>

void MagCalib::putCalibValue(float fDegree, float x, float y)
{
	// rounding to full degrees makes the thing easier
	int iDegree = (int)normalizeAngle360(fDegree);
	auto& entry = this->entries[iDegree];

	entry.x += x;
	entry.y += y;
	++entry.cnt;
}

float MagCalib::finishCalib()
{
	float centerX = 0.f;
	float centerY = 0.f;
	int total = 0;

	// first loop: calc center
	for (int i = 0; i < 360; i++)
	{
		auto& entry = this->entries[i];
		if (entry.cnt > 0)
		{
			entry.x /= (float)entry.cnt;
			entry.y /= (float)entry.cnt;

			Serial.print("Point @ ");
			Serial.print(i);
			Serial.print(" : ");
			Serial.print(entry.x);
			Serial.print(" ");
			Serial.println(entry.y);

			centerX += entry.x;
			centerY += entry.y;
			++total;
		}
	}

	centerX /= (float)total;
	centerY /= (float)total;

	Serial.print("Center: ");
	Serial.print(centerX);
	Serial.print("x");
	Serial.println(centerY);

	// second loop: calc heading
	float fHeading0X = 0.f;
	float fHeading0Y = 0.f;

	for (int i = 0; i < 360; i++)
	{
		auto& entry = this->entries[i];
		if (entry.cnt > 0)
		{
			// calc angle
			entry.x -= centerX;
			entry.y -= centerY;

			float heading = rad2deg(atan2(entry.y, entry.x));

			// calc heading of "0" position
			float heading0 = normalizeAngle360(heading - (float)i);
			float heading0Rad = deg2rad(heading0);

			Serial.print("Heading @ ");
			Serial.print(i);
			Serial.print(" :");
			Serial.print(heading);
			Serial.print(", ");
			Serial.println(heading0);

			// add to sum vector
			fHeading0X += cos(heading0Rad);
			fHeading0Y += sin(heading0Rad);
		}
	}

	float fHeading0Rad = atan2(fHeading0Y, fHeading0X);
	float fHeading0 = rad2deg(fHeading0Rad);

	Serial.print(F("Calib FINAL Heading0: "));
	Serial.print(fHeading0);
	Serial.print(F(" "));
	Serial.println(fHeading0Rad);

	return fHeading0;
}

