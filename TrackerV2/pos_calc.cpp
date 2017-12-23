#include "pos_calc.h"
#include "tools.h"
#include <math.h>

PosCalc::Difference PosCalc::calcDiff(const Position& posFrom, const Position& posTo)
{
	// calc radian lat/lon
	auto lat1 = (double)deg2rad(posFrom.lat);
	auto lon1 = (double)deg2rad(posFrom.lon);
	auto lat2 = (double)deg2rad(posTo.lat);
	auto lon2 = (double)deg2rad(posTo.lon);
	auto R = 6371000.0; // earth radius

	auto deltaLat = (lat2 - lat1);
	auto deltaLon = (lon2 - lon1);

	// distance
	auto a = pow(sin(deltaLat / 2.), 2.) +
		cos(lat1) * cos(lat2) *
		pow(sin(deltaLon / 2.), 2.);
	auto c = 2. * atan2(sqrt(a), sqrt(1. - a));

	Difference ret;
	ret.distance = (float)(R * c);

	// bearing
	auto y = sin(lon2 - lon1) * cos(lat2);
	auto x = cos(lat1) * sin(lat2) -
		sin(lat1) * cos(lat2) * cos(lon2 - lon1);

	ret.bearingDeg = rad2deg((float)(atan2(y, x)));

	// elevation
	if (ret.distance > 0.f)
	{
		float elediff = posTo.alt - posFrom.alt;
		float c2 = sqrtf(powf(ret.distance, 2.f) + powf(elediff, 2.f));
		float tmp = elediff / c2;
		float alpha = asinf(tmp);

		ret.elevationDeg = normalizeAngle360(rad2deg(alpha));
		if (ret.elevationDeg > 180.f)
			ret.elevationDeg -= 360.f;
	}
	return ret;
}

float PosCalc::getMagDeclination(float lat, float lon)
{
	/*
	lookup declination values from 10 x 10 degree grid and return approximate declination for (lat,lon)
	data; 482 declination values (rounded to nearest degree) stored in 482 bytes
	*/

	/*
	DIAGRAM OF 10X10 GRID SQUARE:

	(+60)						(lon,latmin+10,decmax=?)
	l	(lonmin,latmin+10,decNW)|	|		|(lonmin+10,latmin+10,decNE)
	a								 --o--x-----o--
	t									|	|		|
	i									|	|		|
	t									+--x(lon,lat,dec=?)
	u									|	|		|
	d								 --o--x-----o--
	e		(lonmin,latmin,decSW)|	|		|(lonmin+10,latmin,decSE)
	(-60)						(lon,latmin,decmin=?)

	(-180)<- longitude ->(+180)

	o = decs from table, x = calculated decs
	*/


	// -60..0..60 step 10 => 2(6) + 1 = 13 dimensions for lat; -180..0..180 step 10 => 2(18) + 1 = 37 dimensions for lon 	
	signed char dec_tbl[13][37] = \
	{	46, 45, 44, 42, 41, 40, 38, 36, 33, 28, 23, 16, 10, 4, -1, -5, -9, -14, -19, -26, -33, -40, -48, -55, -61, \
		- 66, -71, -74, -75, -72, -61, -25, 22, 40, 45, 47, 46, 30, 30, 30, 30, 29, 29, 29, 29, 27, 24, 18, 11, 3, \
		- 3, -9, -12, -15, -17, -21, -26, -32, -39, -45, -51, -55, -57, -56, -53, -44, -31, -14, 0, 13, 21, 26, \
		29, 30, 21, 22, 22, 22, 22, 22, 22, 22, 21, 18, 13, 5, -3, -11, -17, -20, -21, -22, -23, -25, -29, -35, \
		- 40, -44, -45, -44, -40, -32, -22, -12, -3, 3, 9, 14, 18, 20, 21, 16, 17, 17, 17, 17, 17, 16, 16, 16, 13, \
		8, 0, -9, -16, -21, -24, -25, -25, -23, -20, -21, -24, -28, -31, -31, -29, -24, -17, -9, -3, 0, 4, 7, \
		10, 13, 15, 16, 12, 13, 13, 13, 13, 13, 12, 12, 11, 9, 3, -4, -12, -19, -23, -24, -24, -22, -17, -12, -9, \
		- 10, -13, -17, -18, -16, -13, -8, -3, 0, 1, 3, 6, 8, 10, 12, 12, 10, 10, 10, 10, 10, 10, 10, 9, 9, 6, 0, -6, \
		- 14, -20, -22, -22, -19, -15, -10, -6, -2, -2, -4, -7, -8, -8, -7, -4, 0, 1, 1, 2, 4, 6, 8, 10, 10, 9, 9, 9, \
		9, 9, 9, 8, 8, 7, 4, -1, -8, -15, -19, -20, -18, -14, -9, -5, -2, 0, 1, 0, -2, -3, -4, -3, -2, 0, 0, 0, 1, 3, 5, \
		7, 8, 9, 8, 8, 8, 9, 9, 9, 8, 8, 6, 2, -3, -9, -15, -18, -17, -14, -10, -6, -2, 0, 1, 2, 2, 0, -1, -1, -2, -1, 0, \
		0, 0, 0, 1, 3, 5, 7, 8, 8, 9, 9, 10, 10, 10, 10, 8, 5, 0, -5, -11, -15, -16, -15, -12, -8, -4, -1, 0, 2, 3, 2, 1, 0, \
		0, 0, 0, 0, -1, -2, -2, -1, 0, 3, 6, 8, 6, 9, 10, 11, 12, 12, 11, 9, 5, 0, -7, -12, -15, -15, -13, -10, -7, -3, \
		0, 1, 2, 3, 3, 3, 2, 1, 0, 0, -1, -3, -4, -5, -5, -2, 0, 3, 6, 5, 8, 11, 13, 15, 15, 14, 11, 5, -1, -9, -14, -17, \
		- 16, -14, -11, -7, -3, 0, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -4, -7, -8, -8, -6, -2, 1, 5, 4, 8, 12, 15, 17, 18, 16, \
		12, 5, -3, -12, -18, -20, -19, -16, -13, -8, -4, -1, 1, 4, 6, 8, 9, 9, 9, 7, 3, -1, -6, -10, -12, -11, -9, -5, \
		0, 4, 3, 9, 14, 17, 20, 21, 19, 14, 4, -8, -19, -25, -26, -25, -21, -17, -12, -7, -2, 1, 5, 9, 13, 15, 16, 16, \
		13, 7, 0, -7, -12, -15, -14, -11, -6, -1, 3};

	float decSW, decSE, decNW, decNE, decmin, decmax;
	float lonmin, latmin;
	signed short latmin_index, lonmin_index;
	/* set base point (latmin, lonmin) of grid */

	/* no limits test on lon */
	if (lon == 180) lonmin = 170;
	else lonmin = floor(lon / 10) * 10;

	/* supported lat's -60..60, so range check... */
	if (lat >= 60) latmin = 50;
	else if (lat < -60) latmin = -60;
	else latmin = floor(lat / 10) * 10;

	/* array index = (degrees+[60|180])/10 */
	latmin_index = (60 + latmin) / 10;
	lonmin_index = (180 + lonmin) / 10;

	decSW = dec_tbl[latmin_index][lonmin_index];
	decSE = dec_tbl[latmin_index][lonmin_index + 1];
	decNE = dec_tbl[latmin_index + 1][lonmin_index + 1];
	decNW = dec_tbl[latmin_index + 1][lonmin_index];

	/* approximate declination within the grid using bilinear interpolation */

	decmin = (lon - lonmin) / 10 * (decSE - decSW) + decSW;
	decmax = (lon - lonmin) / 10 * (decNE - decNW) + decNW;
	return   (lat - latmin) / 10 * (decmax - decmin) + decmin;
}
