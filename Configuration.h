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
	const char* getTestFileName();
	uint8_t getFileSpeedMilliseconds();
	uint8_t getLEDPin();

private:
	bool _testValue = true;
	const char* _testFileName = "test.log";
	uint8_t _fileSpeedMilliseconds = 10;
	uint8_t _ledPin = LED_BUILTIN;
};

#endif
