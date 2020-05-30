// MissionMonitor.h

#ifndef _MISSIONMONITOR_h
#define _MISSIONMONITOR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


#include <mavlink_2_ardupilot.h>
#include "MAVLinkEventReceiver.h"

class MissionMonitor : public MAVLinkEventReceiver
{
public:
	MissionMonitor();
	virtual void onHeatbeat( mavlink_heartbeat_t  mavlink_heartbeat );

protected:
	ROVER_MODE _roverMode = ROVER_MODE_INITIALIZING; // Initialize the rover filght mode

};

#endif

