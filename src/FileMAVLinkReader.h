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

class FileMAVLinkReader : public MAVLinkReader
{

public:
	FileMAVLinkReader( const char* mavlinkLogFilePath, MAVLinkEventReceiver& mavlinkEvebtReceiver );

	virtual bool readByte( uint8_t* buffer );
	virtual bool tick();
	virtual void start();

protected:
	const char* _mavlinkLogFilePath;
	File _mavlinkFile;
	unsigned long _previousMAVLinkMilliseconds = 0;
	unsigned long _nextIntervalMAVLinkMilliseconds = 5000;


};

#endif

