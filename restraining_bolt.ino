
/**
  * \details
  * This program is designed for and tested on Teensy 4.1 hardware. It may work on
  * other variations of Arduino hardware if they have enough ram available.
  *
  * \see
  * MAVLink for Dummies: https://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf
  * Arduino with MAVLink https://github.com/tmaxxdd/arduino-with-mavlink
  * MAVLink message documentation https://mavlink.io/en/messages/common.html
  * TaskScheduler https://github.com/arkhipenko/TaskScheduler
  * AdruinoLog https://github.com/thijse/Arduino-Log
  * PWMServo https://github.com/PaulStoffregen/PWMServo
  * TTSAutomate https://ttsautomate.com/
  *
  * \author Vincent Miceli
  *
  * \copyright 2020 Vincent Miceli.
  *
  * \license
  * Licensed under the MIT License <http://opensource.org/licenses/MIT>.
  * Permission is hereby  granted, free of charge, to any  person obtaining a copy
  * of this software and associated  documentation files (the "Software"), to deal
  * in the Software  without restriction, including without  limitation the rights
  * to  use, copy,  modify, merge,  publish, distribute,  sublicense, and/or  sell
  * copies  of  the Software,  and  to  permit persons  to  whom  the Software  is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE  IS PROVIDED "AS  IS", WITHOUT WARRANTY  OF ANY KIND,  EXPRESS OR
  * IMPLIED,  INCLUDING BUT  NOT  LIMITED TO  THE  WARRANTIES OF  MERCHANTABILITY,
  * FITNESS FOR  A PARTICULAR PURPOSE AND  NONINFRINGEMENT. IN NO EVENT  SHALL THE
  * AUTHORS  OR COPYRIGHT  HOLDERS  BE  LIABLE FOR  ANY  CLAIM,  DAMAGES OR  OTHER
  * LIABILITY, WHETHER IN AN ACTION OF  CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE  OR THE USE OR OTHER DEALINGS IN THE
  * SOFTWARE.
  */

  /*
  Library headers
  */
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <SerialFlash.h>

#include <ArduinoLog.h>
#include <TaskScheduler.h>

  /*
  Local headers
  */
#include "LogHelper.h"
#include "Blinker.h"
#include "Configuration.h"
#include "EnumHelper.h"
#include "SerialMAVLinkReader.h"
#include "FileMAVLinkReader.h"
#include "MissionMonitor.h"

constexpr int FAILED_NO_SD = -1;
constexpr int FAILED_NO_TEST_FILE = -2;

constexpr int LOG_LEVEL = LOG_LEVEL_VERBOSE; // Log level
constexpr Stream* LOG_TARGET = &Serial; // Target USB serial port for log messages
constexpr auto CONFIG_FILE_NAME = "config.ini";

bool setupStatus = -1;



//Configuration file settings
Configuration* configuration;

//MAVLink event receiverand reader
MAVLinkEventReceiver* eventReceiver;
MAVLinkReader* mavlinkReader;

// Scheduler
Scheduler scheduler;
Task blinkTask;
Task readMAVLinkTask;
Task missionMonitorTask;

//Blinker
Blinker blinker;


/**
* @Brief  
* Initialize logging, onboard LED, and start task scheduler
*
*/
void setup()
{

	AudioMemory( 40 );

	// Inialize onboard LED
	pinMode( LED_BUILTIN, OUTPUT );

	/// Serial debug logging setup	
	Serial.begin( 115200 );

	Log.begin( LOG_LEVEL, LOG_TARGET, false );
	Log.setSuffix( printNewline );
	Log.trace( "" ); // Create a new line before starting timestamp
	Log.setPrefix( printTimestamp );


	// Check for SD Card
	if ( !SD.begin( BUILTIN_SDCARD ) )
	{
		Log.error( "Could not read SD card" );
		SetupFailed( FAILED_NO_SD );
		return;
	}
	else
	{
		Log.trace( "Found SD card" );

		// Read configuration if it exists
		configuration = new Configuration();

		if ( ! configuration->init( CONFIG_FILE_NAME ) )
		{
			Log.trace( "Using defaults, configuration file not found: %s", CONFIG_FILE_NAME );
		}
		else
		{
			Log.trace( "Loaded configuration file: %s", CONFIG_FILE_NAME );
		}


		// Setup the mavlink reader and monitor
		eventReceiver = new MissionMonitor(configuration->getSecondsBeforeEmergencyStop());

		if ( configuration->getTesting() == false )
		{
			if ( SD.exists( configuration->getTestFileName() ) )
			{
				Log.trace( "Using MAVLink test file: %s at %d milliseconds per message", configuration->getTestFileName(), configuration->getFileSpeedMilliseconds() );
				Log.trace( "Restraining bolt starting...." );
				mavlinkReader = new FileMAVLinkReader( configuration->getTestFileName(), eventReceiver , configuration->getFileSpeedMilliseconds());

			}
			else
			{
				Log.error( "Cound not find test file: %s", configuration->getTestFileName() );
				SetupFailed( FAILED_NO_TEST_FILE );
				return;

			}
		}
		else
		{
			Log.trace( "Using real time MAVLink over serial 1" );
			Log.trace( "Restraining bolt starting...." );
			mavlinkReader = new SerialMAVLinkReader( &Serial1, eventReceiver );

		}

		/**
		 * @brief 
		 * Running a mission live or from file require to different mission timing solutions.
		 * File uses the recorded time in MAVLink packets while live uses local millis()
		*/
		eventReceiver->setMissionTimeCallback( []() {return mavlinkReader->getMissionTime(); } );


		// Read from MAVLink task
		readMAVLinkTask.set( TASK_MILLISECOND * 1, TASK_FOREVER, &mavlinkReaderTick ); 
		scheduler.addTask( readMAVLinkTask );
		readMAVLinkTask.enable();

		// Run mission task
		missionMonitorTask.set( TASK_MILLISECOND * 250, TASK_FOREVER, &eventReceiverTick );
		scheduler.addTask( missionMonitorTask );
		missionMonitorTask.enable();

		SetupSucceeded();
	}

	

}

/**
 * @brief 
 * Main program loop provides execution thread to scheduler
*/
void loop()
{
	scheduler.execute();

}

/**
 * @brief This method will blink the onboard LED with a pattern for a specific error code
 * @param errorCode The code for the error that occured
*/
void SetupFailed(int errorCode)
{
	// Blink Task
	blinkTask.set( TASK_MILLISECOND * 250, TASK_FOREVER, &blinkTick );
	scheduler.addTask( blinkTask );
	blinkTask.enable();

}

void SetupSucceeded()
{
	// Blink Task
	blinkTask.set( TASK_MILLISECOND * 1000, TASK_FOREVER, &blinkTick );
	scheduler.addTask( blinkTask );
	blinkTask.enable();
}

/**
 * @brief MAVLinkReader callback for scheduler 
*/
void mavlinkReaderTick()
{
	mavlinkReader->tick();
}

/**
 * @brief EventReceiver callback for scheduler
*/
void eventReceiverTick()
{
	eventReceiver->tick();
}

/**
 * @brief Callback for normal LED blinking
*/
void blinkTick()
{
	blinker.tick();
}
