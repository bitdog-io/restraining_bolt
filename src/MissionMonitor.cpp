// 
// 
// 

#include "MissionMonitor.h"

MissionMonitor::MissionMonitor()
{}

void MissionMonitor::onHeatbeat( mavlink_heartbeat_t mavlink_heartbeat )
{
	if ( mavlink_heartbeat.type == (uint8_t)MAV_TYPE_GROUND_ROVER )
		if ( mavlink_heartbeat.custom_mode != (uint32_t)_roverMode )
		{
			Serial.printf( "Rover mode changed from %d to %d\r\n", _roverMode, mavlink_heartbeat.custom_mode );
			_roverMode = (ROVER_MODE)mavlink_heartbeat.custom_mode;
		}
}
