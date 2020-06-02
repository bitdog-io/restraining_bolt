// 
// 
// 
#include <ctime>

#include "MissionMonitor.h"
#include "EnumHelper.h"



MissionMonitor::MissionMonitor()
{}

void MissionMonitor::onHeatbeat( mavlink_heartbeat_t mavlink_heartbeat )
{
	if ( mavlink_heartbeat.type == (uint8_t)MAV_TYPE_GROUND_ROVER )
		if ( mavlink_heartbeat.custom_mode != (uint32_t)_roverMode )
		{
			ROVER_MODE roverMode = (ROVER_MODE)mavlink_heartbeat.custom_mode;
			Log.trace( "Rover mode changed from %s to %s", EnumHelper::convert( _roverMode ), EnumHelper::convert( roverMode ) );

			_roverMode = roverMode;
		}
}

void MissionMonitor::onMissionItemReached( mavlink_mission_item_reached_t mavlink_mission_item_reached )
{
	Log.trace( "Destination reached: %d", mavlink_mission_item_reached.seq );
}

void MissionMonitor::onNavControllerOutput( mavlink_nav_controller_output_t mavlink_nav_controller )
{
	bool progressMade = false;

	if ( _lastDistanceToWaypoint == -1 )
	{
		Log.trace( "Distance to new waypoint is %d", mavlink_nav_controller.wp_dist );
		progressMade = true;

	}
	else if ( _lastDistanceToWaypoint == mavlink_nav_controller.wp_dist )
	{
		Log.trace( "Distance to waypoint is %d", mavlink_nav_controller.wp_dist );
	}
	else if ( _lastDistanceToWaypoint < mavlink_nav_controller.wp_dist )
	{
		Log.trace( "Distance to waypoint is %d and growing for %d milliseconds", mavlink_nav_controller.wp_dist, _systemBootTimeMilliseconds - _lastProgressMadeTimeMilliseconds );
	}
	else
	{
		Log.trace( "Distance to waypoint is %d and closing", mavlink_nav_controller.wp_dist );
		progressMade = true;
	}

	_lastDistanceToWaypoint = mavlink_nav_controller.wp_dist;

	if ( progressMade )
		_lastProgressMadeTimeMilliseconds = _systemBootTimeMilliseconds;
}

void MissionMonitor::onMissionCurrent( mavlink_mission_current_t mavlink_mission_current )
{
	if ( _currentWaypointSequenceId == mavlink_mission_current.seq )
	{
		Log.trace( "Current destination: %d", mavlink_mission_current.seq );
	}
	else
	{
		Log.trace( "New destination: %d", mavlink_mission_current.seq );
		_currentWaypointSequenceId = mavlink_mission_current.seq;
		_lastDistanceToWaypoint = -1;
		_lastProgressMadeTimeMilliseconds = _systemBootTimeMilliseconds;
	}

}

void MissionMonitor::onGPSRawInt( mavlink_gps_raw_int_t mavlink_gps_raw_int )
{
	MAVLinkEventReceiver::onGPSRawInt( mavlink_gps_raw_int );
}

void MissionMonitor::onSystemTime( mavlink_system_time_t mavlink_system_time )
{
	_systemBootTimeMilliseconds = mavlink_system_time.time_boot_ms;
	Log.trace( "System time: %d", _systemBootTimeMilliseconds );
}



void MissionMonitor::tick()
{
	evaluateMission();
}

/**
 * @brief
 * This method will monitor the current state of the flight controller. When the flight controller
 * is in AUTO mode, the logic will ensure progress is being made between waypoints. If it detects a problem, this
 * method will call fail mission.
*/
void MissionMonitor::evaluateMission()
{
	unsigned long timedifference = _systemBootTimeMilliseconds - _lastProgressMadeTimeMilliseconds;

	if ( _roverMode == ROVER_MODE_AUTO && timedifference > 5 * 1000 )
	{
		Log.trace( "*************** SHUTDOWN ******************************" );
		Log.trace( "Last progress time: %d  system time: %d difference: %d", _lastProgressMadeTimeMilliseconds, _systemBootTimeMilliseconds, timedifference );
		Log.trace( "*******************************************************" );

	}
	else
	{
		Log.trace( "All is well with mission" );
	}
}

void MissionMonitor::failMission()
{}



