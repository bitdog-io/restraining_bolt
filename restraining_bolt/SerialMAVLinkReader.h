// SerialMAVLinkReader.h

#ifndef _SERIALMAVLINKREADER_h
#define _SERIALMAVLINKREADER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include "MAVLinkReader.h"

class SerialMAVLinkReader : public MAVLinkReader {


protected:
	HardwareSerial *_serial;
    virtual void requestMAVLinkStreams();
	virtual void sendMAVLinkHeartbeat();

public:
	SerialMAVLinkReader( HardwareSerial* serial, MAVLinkEventReceiver* mavlinkEvebtReceiver );
	virtual void start();
	virtual bool readByte( uint8_t* buffer );

private:

	// Heartbeat timer fields
	const int _numberOfHeartBeatsToWait = 60;              // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
	unsigned long  _previousMAVLinkMilliseconds = 0;        ///< will store last time MAVLink was transmitted and listened
	unsigned long _nextIntervalMAVLinkMilliseconds = 1000;  ///< next interval to count
	int _heartbeatCount = _numberOfHeartBeatsToWait;          ///< Initialize so the first heartbeat message happens right away
	unsigned long _currentMillisMAVLink = 0;                 ///< used for timing heartbeats

	// MAVLink configuration fields
	int _sysid = 4;                          ///< ID 4 for this companion computer. 1 flight controller, 255 ground station
	int _compid = MAV_COMP_ID_PERIPHERAL;    ///< The component sending the message
	int _type = MAV_TYPE_ONBOARD_CONTROLLER; ///< This system is a companion computer

	uint8_t _systemType = MAV_TYPE_ONBOARD_CONTROLLER; ///< System type is onboard compainion computer
	uint8_t _autopilotType = MAV_AUTOPILOT_INVALID;  ///< Autopilot type - not an autopilot

	uint8_t _systemMode = MAV_MODE_AUTO_ARMED; ///< Ready to go
	uint32_t _customMode = 0;                  ///< Custom mode, can be defined by user/adopter
	uint8_t _systemState = MAV_STATE_STANDBY;  ///< System ready for flight
};
#endif

