#include "mavlink_processor.h"
#include <Arduino.h>
#include <mavlink.h>

bool MavlinkProcessor::process(char chr)
{
	mavlink_message_t msg;
	mavlink_status_t status;

	if (mavlink_parse_char(MAVLINK_COMM_0, chr, &msg, &status))
	{
		switch (msg.msgid)
		{
		case MAVLINK_MSG_ID_HEARTBEAT:
			Serial.println("Mavlink heartbeat received");
			break;
		case MAVLINK_MSG_ID_GPS_RAW_INT:
		{
			auto lat = (float)mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
			auto lon = (float)mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;

			if (abs(lat) > 0.01f  && abs(lon) > 0.01f)
			{
				this->lat = lat;
				this->lon = lon;
				this->course = (float)mavlink_msg_gps_raw_int_get_cog(&msg) / 100.0f;
				this->vel = (float)mavlink_msg_gps_raw_int_get_vel(&msg) / 100.0f;
				this->had_fix_1 = true;
				this->last_gps_msg = millis();
			}

			Serial.print("Mavlink GPS pos received: ");
			Serial.print(this->lat);
			Serial.print(" ");
			Serial.println(this->lon);

			break;
		}
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			// with vector AP, the x-fire doesn't return relative altitude (it returns absolute altitude instead)
			// we can detect that case by checking if relative and absolute alt are the same
			auto alt1 = (float)mavlink_msg_global_position_int_get_relative_alt(&msg) / 1000.f;
			auto alt2 = (float)mavlink_msg_global_position_int_get_alt(&msg) / 1000.f;
			if (alt1 != alt2)
			{
				this->alt_diff = 0.f;
			}
			else if (!bHomeAltSet)
			{
				// record home altitude
				this->alt_diff = alt2;
				bHomeAltSet = true;
			}

			this->alt = alt1 - this->alt_diff;
			Serial.print("Mavlink GPS altitude #2: ");
			Serial.println(this->alt);
			this->had_fix_2 = true;
			break;
		}
	}

	//Serial.print(int(chr));
	//Serial.print(" ");
	//Serial.print(mavlink_get_channel_status(MAVLINK_COMM_0)->parse_state);
	//Serial.print(" ");
	//Serial.print(mavlink_get_channel_status(MAVLINK_COMM_0)->parse_error);
	//Serial.print("       ");

	return false;
}

bool MavlinkProcessor::getPos(float& lat, float& lon, float& alt)
{
	if (this->had_fix_1 && this->had_fix_2 && millis() - this->last_gps_msg < 5000)
	{
		lat = this->lat;
		lon = this->lon;
		alt = this->alt;
		return true;
	}
	return false;
}

