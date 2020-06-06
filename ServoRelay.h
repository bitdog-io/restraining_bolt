// ServoRelay.h

#ifndef _SERVORELAY_h
#define _SERVORELAY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <PWMServo.h>



class ServoRelay
{
public:
	ServoRelay();
	void powerRelayOff();
	void powerRelayOn();
	void alarmRelayOff();
	void alarmRelayOn();

private:
	PWMServo _pwmPowerSystemRelay;  
	PWMServo _pwmAlarmRelay;
};
#endif

