// 
// 
// 
#include <ctime>

#include "MissionMonitor.h"
#include "EnumHelper.h"
#include "ArduinoLog.h"
#include "AudioPlayer.h"


MissionMonitor::MissionMonitor()
{}

void MissionMonitor::onHeatbeat( mavlink_heartbeat_t mavlink_heartbeat )
{
	// Check for state change
	if ( mavlink_heartbeat.type == (uint8_t)MAV_TYPE_GROUND_ROVER )
		if ( mavlink_heartbeat.custom_mode != (uint32_t)_roverMode )
		{
			ROVER_MODE roverMode = (ROVER_MODE)mavlink_heartbeat.custom_mode;
			MAV_MODE_FLAG mavModeFlag = (MAV_MODE_FLAG)mavlink_heartbeat.base_mode;

			Log.trace( "Rover mode changed from %s to %s ", EnumHelper::convert( _roverMode ), EnumHelper::convert( roverMode ) );

			_mavModeFlag = mavModeFlag;
			_roverMode = roverMode;

			// If we have gone to auto
			if ( _roverMode == ROVER_MODE_AUTO )
			{
				// reset time monitor
				_lastProgressMadeTimeMilliseconds = 0;
			}

			// Switching modes turns on power
			_servoRelay.powerRelayOn();
			_servoRelay.alarmRelayOff();
		}
}

void MissionMonitor::onMissionItemReached( mavlink_mission_item_reached_t mavlink_mission_item_reached )
{
	Log.trace( "Destination reached: %d", mavlink_mission_item_reached.seq );
}

void MissionMonitor::onNavControllerOutput( mavlink_nav_controller_output_t mavlink_nav_controller )
{
	bool progressMade = false;
	uint32_t missionTime = getMissionTime();

	if ( _lastDistanceToWaypoint == -1 )
	{
		Log.trace( "Distance to new waypoint is %d", mavlink_nav_controller.wp_dist );
		progressMade = true;

	}
	else if ( _lastDistanceToWaypoint == mavlink_nav_controller.wp_dist )
	{
		//Log.trace( "Distance to waypoint is %d Mission Time: %d", mavlink_nav_controller.wp_dist,getMissionTime() );
		progressMade = true;
	}
	else if ( _lastDistanceToWaypoint < mavlink_nav_controller.wp_dist )
	{
		Log.trace( "Distance to waypoint is %d and growing for %d milliseconds", mavlink_nav_controller.wp_dist, missionTime - _lastProgressMadeTimeMilliseconds );
	}
	else
	{
		Log.trace( "Distance to waypoint is %d and closing Mission Tome: %d", mavlink_nav_controller.wp_dist, getMissionTime() );
		progressMade = true;
	}

	_lastDistanceToWaypoint = mavlink_nav_controller.wp_dist;

	if ( progressMade )
		_lastProgressMadeTimeMilliseconds = missionTime;
}

void MissionMonitor::onMissionCurrent( mavlink_mission_current_t mavlink_mission_current )
{
	if ( _currentWaypointSequenceId != mavlink_mission_current.seq )
	{
		Log.trace( "New destination: %d", mavlink_mission_current.seq );
		_currentWaypointSequenceId = mavlink_mission_current.seq;
		_lastDistanceToWaypoint = -1;
		_lastProgressMadeTimeMilliseconds = getMissionTime();
	}

}

void MissionMonitor::onGPSRawInt( mavlink_gps_raw_int_t mavlink_gps_raw_int )
{
	MAVLinkEventReceiver::onGPSRawInt( mavlink_gps_raw_int );
}




void MissionMonitor::tick()
{
	evaluateMission();
	_audioPlayer.tick();
}

/**
 * @brief
 * This method will monitor the current state of the flight controller. When the flight controller
 * is in AUTO mode, the logic will ensure progress is being made between waypoints. If it detects a problem, this
 * method will call fail mission.
*/
void MissionMonitor::evaluateMission()
{
	uint32_t  missionTime = getMissionTime();
	uint32_t timeDifference =  missionTime - _lastProgressMadeTimeMilliseconds;

	if ( _roverMode == ROVER_MODE_AUTO && _lastProgressMadeTimeMilliseconds != 0 && timeDifference > (5 * 1000) )
	{
		Log.trace( "*************** SHUTDOWN *********************************************" );
		Log.trace( "Last progress time: %d  mission time: %d difference: %d", _lastProgressMadeTimeMilliseconds, missionTime, timeDifference );
		Log.trace( "**********************************************************************" );

		failMission();

		_audioPlayer.play( EMERGENCY_STOP_SOUND );

	}
	else if ( _roverMode == ROVER_MODE_AUTO )
	{
		//Log.trace( "All is well with mission" );
	}
}

void MissionMonitor::failMission()
{
	_servoRelay.powerRelayOff();
	_servoRelay.alarmRelayOn();
}



