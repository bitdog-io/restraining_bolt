// 
// 
// 

#include "MAVLinkEventReceiver.h"
#include <ArduinoLog.h>


MAVLinkEventReceiver::MAVLinkEventReceiver()
{

}

void MAVLinkEventReceiver::onHeatbeat( mavlink_heartbeat_t mavlink_heartbeat )
{
	//Log.trace( "Got heatbeat message" );
}

void MAVLinkEventReceiver::onSysStatus( mavlink_sys_status_t mavlink_sys_status )
{
	//Log.trace( "Got system status" );
}

void MAVLinkEventReceiver::onParamValue( mavlink_param_value_t mavlink_param_value )
{
	//Log.trace( "Got param value" );
}

void MAVLinkEventReceiver::onRawIMU( mavlink_raw_imu_t mavlink_raw_imu )
{
	//Log.trace( "Got raw IMU" );
}

void MAVLinkEventReceiver::onGPSInput( mavlink_gps_input_t mavlink_gps_input )
{
	//Log.trace( "Got GPS input" );
}

void MAVLinkEventReceiver::onNavControllerOutput( mavlink_nav_controller_output_t mavlink_nav_controller )
{
	//Log.trace( "Got controller output" );
}

void MAVLinkEventReceiver::onMissionItemReached( mavlink_mission_item_reached_t mavlink_mission_item_reached )
{
	//Log.trace( "Got mission item reached" );
}

void MAVLinkEventReceiver::onGPSRawInt( mavlink_gps_raw_int_t mavlink_gps_raw_int )
{
	//Log.trace( "Got GPS raw" );
}

void MAVLinkEventReceiver::onMissionCurrent( mavlink_mission_current_t mavlink_mission_current )
{
	//Log.trace( "Got mission current" );
}

void MAVLinkEventReceiver::onRCChannels( mavlink_rc_channels_t mavlink_rc_channels )
{
	//Log.trace( "Got rc channels" );
}

void MAVLinkEventReceiver::onSystemTime( mavlink_system_time_t mavlink_system_time )
{
	//Log.trace( "System time: %d", mavlink_system_time.time_boot_ms );
}

void MAVLinkEventReceiver::setMissionTimeCallback( uint32_t( *missionTimeCallback ) () )
{
	_missionTimeCallback = missionTimeCallback;
}





void MAVLinkEventReceiver::tick()
{}

long long MAVLinkEventReceiver::getMissionTime()
{
	if ( _missionTimeCallback == NULL )
	{
		return millis();
	}
	else
	{
		return _missionTimeCallback();
	}
}



