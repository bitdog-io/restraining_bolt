
/**
  * \details

  *
  * \see
  * MAVLink for Dummies: https://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf
  * Arduino with MAVLink https://github.com/tmaxxdd/arduino-with-mavlink
  * MAVLink message documentation https://mavlink.io/en/messages/common.html
  * TaskScheduler https://github.com/arkhipenko/TaskScheduler
  * AdruinoLog https://github.com/thijse/Arduino-Log
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
#include <SD_t3.h>
#include <SD.h>
#include <ArduinoLog.h>
#include <TaskScheduler.h>

#include "Blinker.h"
#include "Configuration.h"
#include "EnumHelper.h"
#include "SerialMAVLinkReader.h"
#include "FileMAVLinkReader.h"
#include "MissionMonitor.h"

constexpr int LOG_LEVEL = LOG_LEVEL_VERBOSE; // Log level
constexpr Stream* LOG_TARGET = &Serial; // Target USB serial port for log messages
constexpr auto CONFIG_FILE_NAME = "config.ini";

Configuration configuration;
bool setupStatus = -1;


// MAVLink event receiver and reader selection
MissionMonitor eventReceiver;


// SerialMAVLinkReader mavlinkReader( &Serial1, eventReceiver );
FileMAVLinkReader mavlinkReader( "test.tlog", eventReceiver );

// Scheduler
Scheduler scheduler;
Task blinkTask;
Task readMAVLinkTask;

//Blinker
Blinker blinker;


/**
* @Brief  Initialize logging, onboard LED, and start task scheduler

*/
void setup()
{

	/// Serial logging setup	
	Serial.begin( 115200 );
	Log.begin( LOG_LEVEL, LOG_TARGET, false );

	Log.setPrefix( printTimestamp );
	Log.setSuffix( printNewline );
	Log.trace( "" );

	Log.trace( "Restraining bolt starting...." );

	if ( !SD.begin( BUILTIN_SDCARD ) )
	{
		Log.error( "Could not read SD card" );
		return;
	}
	else
	{
		Log.trace( "Found SD card" );
	}

	// Read configuration file
	if ( configuration.init(CONFIG_FILE_NAME ))
	{
		Log.trace( "Configuration file not found: %s", CONFIG_FILE_NAME );
	}



	// Blink Task
	blinkTask.set( TASK_SECOND * 1, TASK_FOREVER, &blink );
	scheduler.addTask( blinkTask );
	blinkTask.enable();

	// Read from MAVLink task
	readMAVLinkTask.set( TASK_MILLISECOND * 10, TASK_FOREVER, &readMAVLink );
	scheduler.addTask( readMAVLinkTask );
	readMAVLinkTask.enable();

	setupStatus = 0;

}


void loop()
{
	if ( setupStatus == 0 )
		scheduler.execute();
}


void readMAVLink()
{
	mavlinkReader.tick();
}

void blink()
{
	blinker.tick();
}


/**
 * @brief Log timestamp generator function
 * @param _logOutput The stream to write on
*/
void printTimestamp( Print* _logOutput )
{
	char c[12];
	sprintf( c, "%10lu ", millis() );
	_logOutput->print( c );
}

/**
 * @brief Adds return line feed to the end of log records
 * @param _logOutput The stream to wrtie on
*/
void printNewline( Print* _logOutput )
{
	_logOutput->print( "\r\n" );
	LOG_TARGET->flush();
}
