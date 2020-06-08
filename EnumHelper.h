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
	/**
	 * @brief Convert the enum to string.
	 * @param roverMode The value to convert.
	 * @return String value.
	*/
	static const char* convert( ROVER_MODE roverMode );

	/**
	 * @brief Convert the enum to string.
	 * @param mavModeFag The value to convert.
	 * @return String value.
	*/
	static const char* convert( MAV_MODE_FLAG mavModeFag );
};

#endif

