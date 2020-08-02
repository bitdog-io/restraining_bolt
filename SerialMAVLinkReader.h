#pragma once

// SerialMAVLinkReader.h

#ifndef _SERIALMAVLINKREADER_h
#define _SERIALMAVLINKREADER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
#include "MAVLinkReader.h"



class SerialMAVLinkReader : public MAVLinkReader
{


protected:
	HardwareSerial* _serial;
	virtual void requestMAVLinkStreams();
	virtual void sendMAVLinkHeartbeat();

public:
	/**
	 * @brief Constructor
	 * @param serial The serail interface to receive MAVLink message from.
	 * @param mavlinkEvebtReceiver The event receiver to send captured messages to.
	 * 
	*/
	SerialMAVLinkReader( HardwareSerial* serial, MAVLinkEventReceiver* mavlinkEvebtReceiver );

	/**
	 * @brief Read a single btye from MAVLink serial line.
	 * @param buffer The buffer to copy the byte to.
	 * @return True is a byte was read.
	*/
	virtual bool readByte( uint8_t* buffer );

	/**
	 * @brief Used by the scheduling system to pass execution to the serial MAVLink reader.
	*/
	virtual void tick();

	virtual void sendChangeMode( ROVER_MODE roverMode);

private:

	// Heartbeat timer fields
	const int _numberOfCyclesToWait = 60;              ///< of cycles to wait before activating STREAMS from Pixhawk. 60 = one minute.
	unsigned long  _previousMAVLinkMilliseconds = 0;        ///< will store last time MAVLink was transmitted and listened
	const unsigned long _nextIntervalMAVLinkMilliseconds = 1000;  ///< next interval to count
	int _cycleCount = 60;          ///< Number of cycles before a data request

	// MAVLink configuration fields
	int _sysid = 4;                          ///< ID 4 for this companion computer. 1 flight controller, 255 ground station
	int _compid = MAV_COMP_ID_PERIPHERAL;    ///< The component sending the message
	int _type = MAV_TYPE_ONBOARD_CONTROLLER; ///< This system is a companion computer

	uint8_t _systemType = MAV_TYPE_ONBOARD_CONTROLLER; ///< System type is onboard compainion computer
	uint8_t _autopilotType = MAV_AUTOPILOT_INVALID;  ///< Autopilot type - not an autopilot

	uint8_t _systemMode = MAV_MODE_AUTO_ARMED; ///< Ready to go
	uint32_t _customMode = 0;                  ///< Custom mode, can be defined by user/adopter
	uint8_t _systemState = MAV_STATE_STANDBY;  ///< System ready for flight

	uint8_t _flight_controller_sysid = 1; ///< Id # of the flight controller
	uint8_t _flight_controller_component = 0; ///< Target component, 0 = all

};
#endif

