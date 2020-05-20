
/* Copyright (C) 2020 Vincent Miceli - All Rights Reserved.
 * You may use, distribute, and modify this code under the
 * terms of the CC-BY-4.0 license. The author and publisher make no claims to the 
 * suitability of this software for any application. By using this
 * software the end-user accepts all responsibility and liability for its use.
 *
 * Please visit: https://creativecommons.org/licenses/by/4.0/ to get the latest version of this license.
 */

 /* This program uses Mavlink 2.0 protocol to monitor a flight controller using ArduPilot software. 
  * It has been designed to work on an Adafruit Flora v3 https://www.adafruit.com/product/659 but it 
  * may work on other Arduino base boards with enough memory. Serial port 1 of the Flora PCB must be connected to a serial 
  * port of the flight controller. The serial port of the flight controller must be configured for Mavlink 2.0 
  * protocol with a baud rate of 115400.
  * 
  * GPIO Pin X will produce PWM pulses at 1800hz when status is good. It will produce 900hz or less when a fly away 
  * or other problem is detected.
  * 
  * 5/20/2020 V1
  */

  
#include <mavlink.h>


// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long nextIntervalMAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

// Launch the serial port in setup
void setup() {
  // MAVLink interface start
  Serial.begin(57600);
}

// Loop your program
void loop() {
  // MAVLink config
  /* The default UART header for your MCU */ 
  int sysid = 4;                   /// ID 4 for this ground station. 1 FC, 255 ground station
  int compid =  MAV_COMP_ID_PERIPHERAL;                /// The component sending the message
  int type = MAV_TYPE_ONBOARD_CONTROLLER;         /// This system is a companion computer  
 
  // Define the system type, in this case an airplane -> on-board controller
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= nextIntervalMAVLink) {
    // Timing variables
    previousMillisMAVLink = currentMillisMAVLink;

    Serial.write(buf, len);

    //Mav_Request_Data();
    num_hbs_pasados++;
    if(num_hbs_pasados>=num_hbs) {
      // Request streams from Pixhawk
      request_data();
      num_hbs_pasados=0;
    }
  }

  // Check reception buffer
  receive_data();
}

void request_data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // STREAMS that can be requested
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

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x02,0x05};
    
  for (int i=0; i < maxStreams; i++) {
    /*
     * mavlink_msg_request_data_stream_pack(system_id, component_id, 
     *    &msg, 
     *    target_system, target_component, 
     *    MAV_DATA_STREAM_POSITION, 10000000, 1);
     *    
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     * 
     */
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
  }
}

void receive_data() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while(Serial.available()>0) {
    uint8_t c = Serial.read();

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
             */
            //mavlink_message_t* msg;
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);
          }
          break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);


          }
          break;

        
       default:
          break;
      }
    }
  }
}
