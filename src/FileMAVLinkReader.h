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

class FileMAVLinkReader : public MAVLinkReader
{

public:
	FileMAVLinkReader( char* mavlinkLogFilePath, MAVLinkEventReceiver& mavlinkEvebtReceiver );
	virtual bool tick();

};

#endif

