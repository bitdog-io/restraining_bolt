// Configuration.h

#ifndef _CONFIGURATION_h
#define _CONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <SDConfigFile.h>

/**
 * @brief Configuration reads configratuon file from SD card.
*/
class Configuration
{
public:
	Configuration();

	/**
	 * @brief  Initialize configuration with file found on SD card.
	 *
	 * @param configurationFilePath The path to the configuration file.
	 *
	 * @return True if file is found
	*/
	bool init( const char* configurationFilePath );
	
	/**
	 * @brief Read the testing value that was retrieved from the config file.
	 * @return The value retrieved.
	*/
	bool getTesting();

	/**
	 * @brief Read the testFileName value that was retrieved from the config file.
	 * @return The value retrieved.
	*/
	const char* getTestFileName();

	/**
     * @brief Read the fileSpeedMilliseconds value that was retrieved from the config file.
     * @return The value retrieved.
    */
	uint8_t getFileSpeedMilliseconds();

	/**
     * @brief Read the secondsBeforeEmergencyStop value that was retrieved from the config file.
     * @return The value retrieved.
    */
	uint32_t getSecondsBeforeEmergencyStop();

	uint8_t getLowestGPSFixType();

private:
	bool _testing = false;
	const char* _testFileName = "test.log";
	uint8_t _fileSpeedMilliseconds = 10; ///< How fast to read the evetns from file. Don't go lower than 10 else the mission monitor will not keep up.
	uint32_t _secondsBeforeEmergencyStop = 10;
	uint8_t _lowestGPSFixType = 5;
};

#endif
