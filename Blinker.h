// Blinker.h

#ifndef _BLINKER_h
#define _BLINKER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

/**
 * @brief Blinker toggles the onboard LED. 
*/
class Blinker
{
public:
	Blinker();

	/**
	 * @brief Used by the scheduling system to give blinker execution time.
	*/
	void tick();

private:
	void blink();
	bool _isOn = false;
};

#endif

