/**
 * Copyright (C) 2020 Vincent Miceli - All Rights Reserved.
 * You may use, distribute, and modify this code under the
 * terms of the CC-BY-4.0 license. The author and publisher make no claims to the
 * suitability of this software for any application. By using this
 * software the end-user accepts all responsibility and liability for its use.
 * Please visit: https://creativecommons.org/licenses/by/4.0/ to get the latest version of this license.
 *
 *
 * \author Vincent Miceli
 *
 * \copyright  CC-BY-4.0
 *
 */

#include "MAVLinkReader.h"



MAVLinkReader::MAVLinkReader( MAVLinkEventReceiver* mavlinkEventReceiver )
{
	_mavlinkEventReceiver = mavlinkEventReceiver;
}


bool MAVLinkReader::receiveMAVLinkMessages()
{

	uint8_t byteBuffer = 0;

	while ( readByte( &byteBuffer ) )
	{
		mavlink_message_t mavlinkMessage;
		mavlink_status_t status;

		// Try to get a new message
		if ( mavlink_parse_char( MAVLINK_COMM_0, byteBuffer, &mavlinkMessage, &status ) == MAVLINK_FRAMING_OK )
		{
			// Handle message
			switch ( mavlinkMessage.msgid )
			{
				case MAVLINK_MSG_ID_HEARTBEAT: // #0: Heartbeat
					{

						mavlink_heartbeat_t heartbeat;
						mavlink_msg_heartbeat_decode( &mavlinkMessage, &heartbeat );

						_mavlinkEventReceiver->onHeatbeat( heartbeat );

					}
					break;
				case MAVLINK_MSG_ID_SYSTEM_TIME: // #2: SYSTEM_TIME
					{
						mavlink_system_time_t system_time;
						mavlink_msg_system_time_decode( &mavlinkMessage, &system_time );
						_systemBootTimeMilliseconds = system_time.time_boot_ms;
						_mavlinkEventReceiver->onSystemTime( system_time );
					}
					break;

				case MAVLINK_MSG_ID_SYS_STATUS: // #1: SYS_STATUS
					{
						mavlink_sys_status_t sys_status;
						mavlink_msg_sys_status_decode( &mavlinkMessage, &sys_status );

						_mavlinkEventReceiver->onSysStatus( sys_status );
					}
					break;

				case MAVLINK_MSG_ID_PARAM_VALUE: // #22: PARAM_VALUE
					{
						mavlink_param_value_t param_value;
						mavlink_msg_param_value_decode( &mavlinkMessage, &param_value );

						_mavlinkEventReceiver->onParamValue( param_value );
					}
					break;

				case MAVLINK_MSG_ID_RAW_IMU: // #27: RAW_IMU
					{
						mavlink_raw_imu_t imuRaw;
						mavlink_msg_raw_imu_decode( &mavlinkMessage, &imuRaw );

						_mavlinkEventReceiver->onRawIMU( imuRaw );
					}
					break;

				case MAVLINK_MSG_ID_GPS_RAW_INT: // 24
					{
						mavlink_gps_raw_int_t gpsRaw;
						mavlink_msg_gps_raw_int_decode( &mavlinkMessage, &gpsRaw );

						_mavlinkEventReceiver->onGPSRawInt( gpsRaw );

					}
					break;

				case MAVLINK_MSG_ID_GPS_INPUT: // 232
					{
						mavlink_gps_input_t  gpsInput;
						mavlink_msg_gps_input_decode( &mavlinkMessage, &gpsInput );

						_mavlinkEventReceiver->onGPSInput( gpsInput );
					}
					break;


				case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: // #62
					{
						mavlink_nav_controller_output_t navOutput;
						mavlink_msg_nav_controller_output_decode( &mavlinkMessage, &navOutput );

						_mavlinkEventReceiver->onNavControllerOutput( navOutput );
					}
					break;

				case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
					{
						mavlink_mission_item_reached_t itemReached;
						mavlink_msg_mission_item_reached_decode( &mavlinkMessage, &itemReached );

						_mavlinkEventReceiver->onMissionItemReached( itemReached );

					}
					break;

				case MAVLINK_MSG_ID_MISSION_CURRENT:
					{
						mavlink_mission_current_t current;
						mavlink_msg_mission_current_decode( &mavlinkMessage, &current );

						_mavlinkEventReceiver->onMissionCurrent( current );

					}
					break;
				case MAVLINK_MSG_ID_RC_CHANNELS:
					{
						mavlink_rc_channels_t rcChannels;
						mavlink_msg_rc_channels_decode( &mavlinkMessage, &rcChannels );

						_systemBootTimeMilliseconds = rcChannels.time_boot_ms;
						_mavlinkEventReceiver->onRCChannels( rcChannels );
					}
					break;
				default:
					//Log.trace("Got unhandled message id: %d", mavlinkMessage.msgid);
					break;

			}

			return true;

		}

	}

	return false;
}

void MAVLinkReader::tick()
{
	
}

uint32_t MAVLinkReader::getMissionTime()
{

	return millis();

}


bool MAVLinkReader::readByte( uint8_t* buffer )
{
	return false;
}
