#ifndef POS_CALC_H
#define POS_CALC_H

class PosCalc
{
public:
	struct Position
	{
		inline Position(float lat, float lon, float alt)
			: lat(lat), lon(lon), alt(alt) {}

		Position() = default;

		float lat = 0.f;
		float lon = 0.f;
		float alt = 0.f;
	};

	struct Difference
	{
		float distance = 0.f;
		float bearingDeg;
		float elevationDeg = 0.f;
	};

	static Difference calcDiff(const Position& posFrom, const Position& posTo);

	static float getMagDeclination(float lat, float lon);

};


#endif


