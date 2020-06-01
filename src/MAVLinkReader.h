#pragma once

// MAVLinkReader.h

#ifndef _MAVLINKREADER_h
#define _MAVLINKREADER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "MAVLinkEventReceiver.h"

class MAVLinkReader
{

public:
	MAVLinkReader( MAVLinkEventReceiver& mavlinkEventReceiver );

	virtual void start() = 0;
	virtual bool receiveMAVLinkMessages();


protected:
	virtual bool readByte( uint8_t* buffer ) = 0;
	virtual bool tick() = 0;


private:
	MAVLinkEventReceiver* _mavlinkEventReceiver;
};

#endif

