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
#include <ArduinoLog.h>


SerialMAVLinkReader::SerialMAVLinkReader( HardwareSerial* serial, MAVLinkEventReceiver* mavlinkEvebtReceiver )
	: MAVLinkReader( mavlinkEvebtReceiver )
{
	_serial = serial;
	
	Log.trace( "Starting MAVLink serial reader" );
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

void SerialMAVLinkReader::tick()
{
	unsigned long currentMillisMAVLink = millis();

	receiveMAVLinkMessages();

	// If ready to send heartbeat
	if ( currentMillisMAVLink - _previousMAVLinkMilliseconds >= _nextIntervalMAVLinkMilliseconds )
	{

		_previousMAVLinkMilliseconds = currentMillisMAVLink;

		sendMAVLinkHeartbeat();

		_cycleCount += 1;

		// If ready to send data request
		if ( _cycleCount >= _numberOfCyclesToWait )
		{
			// Request streams from Pixhawk
			Log.trace( "Requesting stream data" );
			requestMAVLinkStreams();
			_cycleCount = 0;
		}


	}

}

void SerialMAVLinkReader::requestMAVLinkStreams()
{
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	uint16_t messageLength = 0;
	const int maxStreams = 1;
	uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_ALL };
	uint16_t MAVRates[maxStreams] = { 0x02 };
	mavlink_message_t mavlinkMessage;

	/*
	 * Definitions are in common.h: enum MAV_DATA_STREAM
	 *
	 *
	 * Data in PixHawk available in:
	 *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
	 *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
	 */
	for ( int i = 0; i < maxStreams; i++ )
	{
		mavlink_msg_request_data_stream_pack( _sysid, _compid, &mavlinkMessage, 1, 0, MAVStreams[i], MAVRates[i], 1 );
		messageLength = mavlink_msg_to_send_buffer( buffer, &mavlinkMessage );
		_serial->write( buffer, messageLength );
	}
}

void SerialMAVLinkReader::sendMAVLinkHeartbeat()
{
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	uint16_t messageLength = 0;
	mavlink_message_t mavlinkMessage;

	//Log.trace( "Sending heartbeat message" );

	// Pack the MAVLink heartbeat message
	mavlink_msg_heartbeat_pack( _sysid, _compid, &mavlinkMessage, _type, _autopilotType, _systemMode, _customMode, _systemState );

	// Copy the message to the send buffer
	messageLength = mavlink_msg_to_send_buffer( buffer, &mavlinkMessage );

	// Write buffer containing heartbeat message
	_serial->write( buffer, messageLength );


}







