// EnumHelper.h

#ifndef _ENUMHELPER_h
#define _ENUMHELPER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <mavlink_2_ardupilot.h>

class EnumHelper
{
public:
	static const char* convert( ROVER_MODE roverMode );
};

#endif

