// LogHelper.h

#ifndef _LOGHELPER_h
#define _LOGHELPER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
void printTimestamp( Print* _logOutput );
void printNewline( Print* _logOutput );

#endif

