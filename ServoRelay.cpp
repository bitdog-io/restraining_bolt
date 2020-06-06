// 
// 
// 

#include "ServoRelay.h"
#include <ArduinoLog.h>

constexpr int POWER_SYSTEM_RELAY_PIN = 33;
constexpr int ALARM_RELAY_PIN = 36;
constexpr int ON = 180;
constexpr int OFF = 0;

ServoRelay::ServoRelay()
{
	_pwmPowerSystemRelay.attach( POWER_SYSTEM_RELAY_PIN );
	_pwmPowerSystemRelay.attach( ALARM_RELAY_PIN );
}

void ServoRelay::powerRelayOff()
{
	Log.trace( "Turning off power" );
	_pwmPowerSystemRelay.write( OFF );
}

void ServoRelay::powerRelayOn()
{
	Log.trace( "Turning on power" );
	_pwmPowerSystemRelay.write( ON );
}

void ServoRelay::alarmRelayOff()
{
	Log.trace( "Turning off alarm" );
	_pwmAlarmRelay.write( OFF );
}

void ServoRelay::alarmRelayOn()
{
	Log.trace( "Turning on alarm" );
	_pwmAlarmRelay.write( ON );
}
