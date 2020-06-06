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
	MAVLinkReader( MAVLinkEventReceiver* mavlinkEventReceiver );

	virtual void start() {};
	virtual bool receiveMAVLinkMessages();
	virtual bool tick();
	virtual uint32_t getMissionTime();

protected:
	virtual bool readByte( uint8_t* buffer );
	uint32_t _systemBootTimeMilliseconds = 0;

private:
	MAVLinkEventReceiver* _mavlinkEventReceiver;

};

#endif

