// Configuration.h

#ifndef _CONFIGURATION_h
#define _CONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <SDConfigFile.h>

class Configuration
{
public:
	Configuration();
	bool init( const char* configurationFilePath );
	
	bool getTestValue();


private:
	bool _testValue = false;
};

#endif
