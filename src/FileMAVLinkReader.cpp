#include "FileMAVLinkReader.h"

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
FileMAVLinkReader::FileMAVLinkReader( char* mavlinkLogFilePath,  MAVLinkEventReceiver& mavlinkEvebtReceiver )
	: MAVLinkReader( mavlinkEvebtReceiver )
{}

bool FileMAVLinkReader::tick()
{
	return false;
}
