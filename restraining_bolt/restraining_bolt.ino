/* Copyright (C) 2020 Vincent Miceli - All Rights Reserved.
 * You may use, distribute, and modify this code under the
 * terms of the CC-BY-4.0 license. The author and publisher make no claims to the 
 * suitability of this software for any application. By using this
 * software the end-user accepts all responsibility and liability for its use.
 *
 * Please visit: https://creativecommons.org/licenses/by/4.0/ to get the latest version of this license.
 */

/* This program uses MAVLink 2.0 protocol to monitor a flight controller using ArduPilot software. 
  * It has been designed to work on an Adafruit Flora v3 https://www.adafruit.com/product/659 but it 
  * may work on other Arduino base boards with enough memory. Serial port 1 of the Flora PCB must be connected to a serial 
  * port of the flight controller. The serial port of the flight controller must be configured for Mavlink 2.0 
  * protocol with a baud rate of 115200.
  * 
  * References used:
  * MAVLink for Dummies: https://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf
  * Arduino with MAVLink https://github.com/tmaxxdd/arduino-with-mavlink
  * 
  * GPIO Pin X will produce PWM pulses at 1800hz when status is good. It will produce 900hz or less when a fly away 
  * or other problem is detected.
  * 
  * 5/20/2020 V1
  */
#include <Arduino.h>
#include <ArduinoLog.h>
#include <Adafruit_NeoPixel.h>
#include <mavlink2.h>

#define NEOPIXELPIN 8
#define NEOPIXELCOUNT 1
#define LEDPIN 7

Adafruit_NeoPixel neopixels;

// MavLink communication variables
unsigned long previousMAVLinkMilliseconds = 0;        // will store last time MAVLink was transmitted and listened
unsigned long nextIntervalMAVLinkMilliseconds = 1000; // next interval to count
const int numberOfHeartBeatsToWait = 60;              // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int heartbeatCount = numberOfHeartBeatsToWait;        // Initialize so the first heartbeat message happens right away
unsigned long currentMillisMAVLink = 0;               // used for timing heartbeats

// MAVLink config
int sysid = 4;                          // ID 4 for this companion computer. 1 pixhawk, 255 ground station
int compid = MAV_COMP_ID_PERIPHERAL;    // The component sending the message
int type = MAV_TYPE_ONBOARD_CONTROLLER; // This system is a companion computer

// Define the system type -> on-board controller
uint8_t system_type = MAV_TYPE_ONBOARD_CONTROLLER;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

// Initialize mavlink state and mode
uint8_t system_mode = MAV_MODE_AUTO_ARMED; // Ready to go
uint32_t custom_mode = 0;                  // Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY;  // System ready for flight

// Initialize the required mavlink protocol buffers
mavlink_message_t mavlinkMessage;
mavlink_status_t status;
uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
uint16_t len = 0;
HardwareSerial SerialMAV = Serial1;


/*
* Setup 
* Initialize onboard neopixel, onboard led, and serial ports
*/
void setup()
{

  // Initialize onboard neopixel
  neopixels = Adafruit_NeoPixel(NEOPIXELCOUNT, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);
  neopixels.begin();
  neopixels.setBrightness(90);
  neopixels.show();

  // Inialize onboard led
  pinMode(LEDPIN, OUTPUT);

  // Set up USB serial port
  Serial.begin(115200);
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  // MAVLink interface start
  SerialMAV.begin(115200);
}

void trace(const char *text)
{
  Log.notice("%s%s", text, CR);
}

void blink()
{
  digitalWrite(LEDPIN, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(250);                // wait for a second
  digitalWrite(LEDPIN, LOW);  // turn the LED off by making the voltage LOW
  delay(250);                // wait for a second
}

void loop()
{

  blink();
  // Get run time in milliseconds
  currentMillisMAVLink = millis();

  // check if enough time has past for next MAVLink heatbeat message
  if (currentMillisMAVLink - previousMAVLinkMilliseconds >= nextIntervalMAVLinkMilliseconds)
  {

    // Store the current time for next loop
    previousMAVLinkMilliseconds = currentMillisMAVLink;

    // Pack the MAVLink heartbeat message
    mavlink_msg_heartbeat_pack(sysid, compid, &mavlinkMessage, type, autopilot_type, system_mode, custom_mode, system_state);

    // Copy the message to the send buffer
    len = mavlink_msg_to_send_buffer(buffer, &mavlinkMessage);

    // Write buffer containing heartbeat message
    trace("writing heartbeat message to MAVLink");
    SerialMAV.write(buffer, len);

    heartbeatCount++;
    if (heartbeatCount >= numberOfHeartBeatsToWait)
    {
      // Request streams from Pixhawk
      trace("requesting data streams from flight controller");
      requestMavLinkData();
      heartbeatCount = 0;
    }
  }

  // Check reception buffer
  receiveMavLinkData();

  //rainbowCycle(5);
}

void requestMavLinkData()
{

  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the flight controller
  const int maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x02, 0x05};

  for (int i = 0; i < maxStreams; i++)
  {
    mavlink_msg_request_data_stream_pack(2, 200, &mavlinkMessage, 1, 0, MAVStreams[i], MAVRates[i], 1);
    len = mavlink_msg_to_send_buffer(buffer, &mavlinkMessage);
    SerialMAV.write(buffer, len);
  }
}

void receiveMavLinkData()
{

  trace("checking for MAVLink data");

  while (SerialMAV.available() > 0)
  {
    uint8_t c = SerialMAV.read();

    trace("received message over MAVLink");


    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &mavlinkMessage, &status) == MAVLINK_FRAMING_OK)
    {
/*
      // Handle message
      switch (mavlinkMessage.msgid)
      {
      case MAVLINK_MSG_ID_HEARTBEAT: // #0: Heartbeat
      {
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(&mavlinkMessage, &heartbeat);
      }
      break;

      case MAVLINK_MSG_ID_SYS_STATUS: // #1: SYS_STATUS
      {
        mavlink_sys_status_t sys_status;
        mavlink_msg_sys_status_decode(&mavlinkMessage, &sys_status);
      }
      break;

      case MAVLINK_MSG_ID_PARAM_VALUE: // #22: PARAM_VALUE
      {
        mavlink_param_value_t param_value;
        mavlink_msg_param_value_decode(&mavlinkMessage, &param_value);
      }
      break;

      case MAVLINK_MSG_ID_RAW_IMU: // #27: RAW_IMU
      {
        mavlink_raw_imu_t raw_imu;
        mavlink_msg_raw_imu_decode(&mavlinkMessage, &raw_imu);
      }
      break;

      case MAVLINK_MSG_ID_ATTITUDE: // #30
      {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&mavlinkMessage, &attitude);
      }
      break;

      default:
        break;
      } */
    }
   
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait)
{
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++)
  { // 5 cycles of all colors on wheel
    for (i = 0; i < neopixels.numPixels(); i++)
    {
      neopixels.setPixelColor(i, Wheel(((i * 256 / neopixels.numPixels()) + j) & 255));
    }
    neopixels.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return neopixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    return neopixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  else
  {
    WheelPos -= 170;
    return neopixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}
