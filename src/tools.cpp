#include "tools.h"
#include <math.h>

float normalizeAngle(float val) {
	while (val < -M_PI)
		val += 2.f * M_PI;
	while (val >= M_PI)
		val -= 2.f * M_PI;
	return val;
}

float normalizeAngle360(float val) {
	while (val < 0)
		val += 360.f;
	while (val >= 360.f)
		val -= 360.f;
	return val;
}

float rad2deg(float rad)
{
	return normalizeAngle360(180.f * rad / M_PI);
}

float deg2rad(float deg)
{
	return normalizeAngle(deg * M_PI / 180.f);
}