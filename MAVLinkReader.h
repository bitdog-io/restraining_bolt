#pragma once

// MAVLinkReader.h

#ifndef _MAVLINKREADER_h
#define _MAVLINKREADER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "MAVLinkEventReceiver.h"

/**
 * @brief Base class for reading MAVLink message from a byte source. The messages captured create events to be sent to a MAVLinkEventReceiver
*/
class MAVLinkReader
{

public:
	/**
	 * @brief Constructor
	 * @param mavlinkEventReceiver The event receiver to send events to when a message is received.
	 *
	*/
	MAVLinkReader( MAVLinkEventReceiver* mavlinkEventReceiver );

	/**
	 * @brief Base function for reading MAVLink bytes from source
	 * @return True if a complete message has been read from source
	*/
	virtual bool receiveMAVLinkMessages();

	/**
	 * @brief Semd a MavLink message to change the rover mode to the flight controller
	 * @param roverMode 
	*/
	virtual void sendChangeMode( ROVER_MODE roverMode ) {};

	/**
	 * @brief This is used by the scheduling system to give the MAVLink reader execution time
	*/
	virtual void tick();

	/**
	 * @brief Get the mission time in milliseconds
	 * @return Milliseconds for mission time
	*/
	virtual uint32_t getMissionTime();

protected:
	virtual bool readByte( uint8_t* buffer );
	uint32_t _systemBootTimeMilliseconds = 0;

private:
	MAVLinkEventReceiver* _mavlinkEventReceiver;

};

#endif

