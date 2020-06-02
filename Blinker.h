// Blinker.h

#ifndef _BLINKER_h
#define _BLINKER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class Blinker
{
public:
	Blinker();

	void tick();

private:
	void blink();
	bool _isOn = false;
};

#endif

