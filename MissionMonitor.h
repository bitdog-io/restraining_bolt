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
#include "ServoRelay.h"
#include "AudioPlayer.h"

/**
 * @brief Mission monitor receives events from the MAVLink reader and evaluates the condition of a mission. Its purpose is to shutdown the rover if it is off course.
*/
class MissionMonitor : public MAVLinkEventReceiver
{
public:
	MissionMonitor( uint32_t secondsBeforeEmergencyStop );
	virtual void onHeatbeat( mavlink_heartbeat_t  mavlink_heartbeat );
	virtual void onMissionItemReached( mavlink_mission_item_reached_t mavlink_mission_item_reached );
	virtual void onNavControllerOutput( mavlink_nav_controller_output_t mavlink_nav_controller );
	virtual void onMissionCurrent( mavlink_mission_current_t mavlink_mission_current );
	virtual void onGPSRawInt( mavlink_gps_raw_int_t mavlink_gps_raw_int );
	

	/**
     * @brief Used by the scheduling system to give MissionMonitor execution time.
    */
	virtual void tick();

protected:
	/**
	 * @brief Used to determine if the mission has gone off course.
	*/
	virtual void evaluateMission();

	/**
	 * @brief Centrailized logic for stopping the rover during mission failure.
	*/
	virtual void failMission();

	/**
	 * @brief Centralized logic for starting and restarting rover monitoring
	*/
	virtual void start();

	ROVER_MODE _roverMode = ROVER_MODE_INITIALIZING; // Initialize the rover filght mode
	MAV_MODE_FLAG _mavModeFlag = MAV_MODE_FLAG_ENUM_END;
	int16_t _lastDistanceToWaypoint = -1;
	unsigned long _lastProgressMadeTimeMilliseconds = 0;
	uint16_t _currentWaypointSequenceId = 0;
	bool _firstHeartbeat = false;
	bool _firstTick = false;
	bool _isFailed = false;
	bool _wrongDirection = false;
	uint32_t _wrongDirectionCount = 0;
	uint32_t _secondsBeforeEmergencyStop = 20;



private:
	ServoRelay _servoRelay;
	AudioPlayer _audioPlayer;

};

#endif

