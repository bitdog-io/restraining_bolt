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

#include "SerialMAVLinkReader.h"
#include <TaskScheduler.h>

SerialMAVLinkReader::SerialMAVLinkReader( HardwareSerial* serial, MAVLinkEventReceiver* mavlinkEvebtReceiver )
	: MAVLinkReader( mavlinkEvebtReceiver )
{
	_serial = serial;
	start();
}

void SerialMAVLinkReader::start()
{
	_serial->begin( 57600, SERIAL_8N1 );
}

bool SerialMAVLinkReader::readByte( uint8_t* buffer )
{
	if ( _serial->available() > 0 )
	{
		return _serial->readBytes( buffer, 1 ) == 1;
	}

	return false;
}

void SerialMAVLinkReader::requestMAVLinkStreams()
{
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	uint16_t messageLength = 0;

	/*
	 * Definitions are in common.h: enum MAV_DATA_STREAM
	 *
	 * MAV_DATA_STREAM_ALL=0, // Enable all data streams
	 * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
	 * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
	 * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
	 * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
	 * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
	 * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
	 * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
	 * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
	 * MAV_DATA_STREAM_ENUM_END=13,
	 *
	 * Data in PixHawk available in:
	 *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
	 *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
	 */

	 // To be setup according to the needed information to be requested from the flight controller
	const int maxStreams = 1;
	const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_ALL };
	const uint16_t MAVRates[maxStreams] = { 0x02 };
	mavlink_message_t mavlinkMessage;

	for ( int i = 0; i < maxStreams; i++ )
	{
		mavlink_msg_request_data_stream_pack( 2, 200, &mavlinkMessage, 1, 0, MAVStreams[i], MAVRates[i], 1 );
		messageLength = mavlink_msg_to_send_buffer( buffer, &mavlinkMessage );
		_serial->write( buffer, messageLength );
	}
}

void SerialMAVLinkReader::sendMAVLinkHeartbeat()
{
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	uint16_t messageLength = 0;
	mavlink_message_t mavlinkMessage;

	// Store the current time for next loop
	_previousMAVLinkMilliseconds = _currentMillisMAVLink;

	// Pack the MAVLink heartbeat message
	mavlink_msg_heartbeat_pack( sysid, compid, &mavlinkMessage, type, autopilot_type, system_mode, custom_mode, system_state );

	// Copy the message to the send buffer
	messageLength = mavlink_msg_to_send_buffer( buffer, &mavlinkMessage );

	// Write buffer containing heartbeat message
	_serial->write( buffer, messageLength );


}




