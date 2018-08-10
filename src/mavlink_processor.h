#ifndef MAVLINK_PROCESSOR_H
#define MAVLINK_PROCESSOR_H

class MavlinkProcessor
{
public:
	/**
	 * @return true if a GPS position has been decoded
	 */
	bool process(char chr);

	/**
	 * Gets a position. This may as well be an estimate.
	 */
	bool getPos(float& lat, float& lon, float& alt);

private:
	bool had_fix_1 = false;
	bool had_fix_2 = false;
	long last_gps_msg = 0;
	float lat = 0.f;
	float lon = 0.f;
	float course = 0.f;
	float vel = 0.f;
	float alt = 0.f;

	bool bHomeAltSet = false;
	float alt_diff = 0.f;
};


#endif
