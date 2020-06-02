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
	virtual void onMissionItemReached( mavlink_mission_item_reached_t mavlink_mission_item_reached );
	virtual void onNavControllerOutput( mavlink_nav_controller_output_t mavlink_nav_controller );
	virtual void onMissionCurrent( mavlink_mission_current_t mavlink_mission_current );

	virtual void tick();

protected:
	virtual void evaluateMission();
	virtual void failMission();
	ROVER_MODE _roverMode = ROVER_MODE_INITIALIZING; // Initialize the rover filght mode
	uint16_t _distanceToWaypoint = 0;
	uint16_t _currentWaypointSequenceId = 0;


};

#endif

