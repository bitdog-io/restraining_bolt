// MAVLinkEventReceiver.h

#ifndef _MAVLINKEVENTRECEIVER_h
#define _MAVLINKEVENTRECEIVER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <mavlink_2_ardupilot.h>


class MAVLinkEventReceiver {

public:
	MAVLinkEventReceiver();

	void OnHeatbeat( mavlink_heartbeat_t  mavlink_heartbeat );
	void OnSysStatus( mavlink_sys_status_t  mavlink_sys_status );
	void OnParamValue( mavlink_param_value_t  mavlink_param_value );
	void OnRawIMU( mavlink_raw_imu_t mavlink_raw_imu );
	void OnGPSInput( mavlink_gps_input_t mavlink_gps_input );
	void OnNavControllerOutput( mavlink_nav_controller_output_t mavlink_nav_controller );
	void OnMissionItemReached( mavlink_mission_item_reached_t mavlink_mission_item_reached );
    void OnGPSRawInt( mavlink_gps_raw_int_t mavlink_gps_raw_int );


};

#endif

