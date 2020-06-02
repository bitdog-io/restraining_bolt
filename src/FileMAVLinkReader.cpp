#include "FileMAVLinkReader.h"
#include <ArduinoLog.h>

/**
 * @brief FileMAVLinkReader constructor
 *
 *
 * @param mavlinkLogFilePath File path on SD card to MAVLink log file like what is created by Mission Planner
 *
 * @param mavlinkEvebtReceiver Custom MAVLink receiver to capture incomming events
 *
 *
*/
FileMAVLinkReader::FileMAVLinkReader( const char* mavlinkLogFilePath, MAVLinkEventReceiver* mavlinkEvebtReceiver )
	: MAVLinkReader( mavlinkEvebtReceiver )
{
	_mavlinkLogFilePath = mavlinkLogFilePath;
}

void FileMAVLinkReader::start()
{
	if ( !SD.exists( _mavlinkLogFilePath ) )
	{
		Log.trace( "Cannot find file: %s", _mavlinkLogFilePath );
	}
}

bool FileMAVLinkReader::readByte( uint8_t* buffer )
{
	return _mavlinkFile.readBytes( buffer, 1 ) == 1;
}

bool FileMAVLinkReader::tick()
{
	unsigned long currentMillisMAVLink = millis();

	// If ready to read next message
	if ( currentMillisMAVLink - _previousMAVLinkMilliseconds >= _nextIntervalMAVLinkMilliseconds )
	{
		receiveMAVLinkMessages();
		_previousMAVLinkMilliseconds = currentMillisMAVLink;


	}

	return true;
}



