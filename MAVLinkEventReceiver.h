// MAVLinkEventReceiver.h

#ifndef _MAVLINKEVENTRECEIVER_h
#define _MAVLINKEVENTRECEIVER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <mavlink_2_ardupilot.h>


class MAVLinkEventReceiver
{

public:
	MAVLinkEventReceiver();

	virtual void onHeatbeat( mavlink_heartbeat_t  mavlink_heartbeat );
	virtual void onSysStatus( mavlink_sys_status_t  mavlink_sys_status );
	virtual void onParamValue( mavlink_param_value_t  mavlink_param_value );
	virtual void onRawIMU( mavlink_raw_imu_t mavlink_raw_imu );
	virtual void onGPSInput( mavlink_gps_input_t mavlink_gps_input );
	virtual void onNavControllerOutput( mavlink_nav_controller_output_t mavlink_nav_controller );
	virtual void onMissionItemReached( mavlink_mission_item_reached_t mavlink_mission_item_reached );
	virtual void onGPSRawInt( mavlink_gps_raw_int_t mavlink_gps_raw_int );
	virtual void onMissionCurrent( mavlink_mission_current_t mavlink_mission_current );
	virtual void onRCChannels( mavlink_rc_channels_t mavlink_rc_channels );
	virtual void onSystemTime( mavlink_system_time_t mavlink_system_time );
	virtual void setMissionTimeCallback( uint32_t( *missionTimeCallback ) () );

	virtual void tick();

protected:
	long long getMissionTime();
	uint32_t( *_missionTimeCallback ) ();

};

#endif

