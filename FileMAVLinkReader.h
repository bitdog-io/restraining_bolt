#pragma once

// FileMAVLinkReader.h

#ifndef _FILEMAVLINKREADER_h
#define _FILEMAVLINKREADER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "MAVLinkReader.h"
#include <SD.h>

/**
 * @brief This class reads a telemetry file from SD card for testing. Mission Planner .tlog has been tested.
*/
class FileMAVLinkReader : public MAVLinkReader
{

public:
	/**
	 * @brief Constructor.
	 * @param mavlinkLogFilePath The file to MAVLink file. Use 8.3 file naming convention to support SD API.
	 * @param mavlinkEvebtReceiver The event receiver that will capture MAVLink messages read from the file.
	 * @param fileSpeedMilliseconds The speed at which to read the MAVLink file during testing.
	 * 
	*/
	FileMAVLinkReader( const char* mavlinkLogFilePath, MAVLinkEventReceiver* mavlinkEvebtReceiver, uint8_t fileSpeedMilliseconds );

	/**
	 * @brief Read a single byte from the MAVLink source.
	 * @param buffer A buffer to read the byte into.
	 * @return True if a byte was read.
	*/
	virtual bool readByte( uint8_t* buffer );

	/**
	 * @brief Used by the scheduling system to give FileMAVLinkReader execution time.
	*/
	virtual void tick();

	/**
	 * @brief Returns the latest run time as reported from the flight controller messages in the recorded MAVLink file.
	 * Local millis cannot be used because the file is read at a speed that doesn't represent real time.
	 * @return Milliseconds since flight controller started.
	*/
	virtual uint32_t getMissionTime();

protected:
	const char* _mavlinkLogFilePath;
	File _mavlinkFile;
	unsigned long _previousMAVLinkMilliseconds = 0;
	unsigned long _nextIntervalMAVLinkMilliseconds = 1;


};

#endif
