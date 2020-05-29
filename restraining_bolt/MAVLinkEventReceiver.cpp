// 
// 
// 

#include "MAVLinkEventReceiver.h"


MAVLinkEventReceiver::MAVLinkEventReceiver( ) 
{

}

void MAVLinkEventReceiver::OnHeatbeat( mavlink_heartbeat_t mavlink_heartbeat )
{
	Log.trace( "Got heatbeat message" );
}

void MAVLinkEventReceiver::OnSysStatus( mavlink_sys_status_t mavlink_sys_status )
{
	Log.trace( "Got system status" );
}

void MAVLinkEventReceiver::OnParamValue( mavlink_param_value_t mavlink_param_value )
{
	Log.trace( "Got param value" );
}

void MAVLinkEventReceiver::OnRawIMU( mavlink_raw_imu_t mavlink_raw_imu )
{
	Log.trace( "Got raw IMU" );
}

void MAVLinkEventReceiver::OnGPSInput( mavlink_gps_input_t mavlink_gps_input )
{
	Log.trace( "Got GPS input" );
}

void MAVLinkEventReceiver::OnNavControllerOutput( mavlink_nav_controller_output_t mavlink_nav_controller )
{
	Log.trace( "Got controller output" );
}

void MAVLinkEventReceiver::OnMissionItemReached( mavlink_mission_item_reached_t mavlink_mission_item_reached )
{
	Log.trace( "Got mission item reached" );
}

void MAVLinkEventReceiver::OnGPSRawInt( mavlink_gps_raw_int_t mavlink_gps_raw_int )
{
	Log.trace( "Got GPS raw" );
}



