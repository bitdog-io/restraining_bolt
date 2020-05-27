
/**
  * \details 
  * This program uses MAVLink 2.0 protocol to monitor a flight controller using ArduPilot software.
  * It has been designed to work on an Teensy 4.1 but it may work on other Arduino
  * base boards with enough memory. Serial port 1 of the Teensy PCB must be connected to a serial
  * port of the flight controller. The serial port of the flight controller must be configured for Mavlink 2.0
  * protocol with a baud rate of 57600. 
  * GPIO Pin X will produce PWM pulses at 1800hz when status is good. It will produce 900hz or less when a fly away
  * or other problem is detected.
  *
  * \see
  * MAVLink for Dummies: https://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf
  * Arduino with MAVLink https://github.com/tmaxxdd/arduino-with-mavlink
  * MAVLink message documentation https://mavlink.io/en/messages/common.html
  * TaskScheduler https://github.com/arkhipenko/TaskScheduler
  * \author Vincent Miceli
  *
  * \copyright  CC-BY-4.0
  *
  */


#include <SD_t3.h>
#include <SD.h>
#include <mavlink_2_ardupilot.h>




constexpr auto LEDPIN = 13;

// Test File
File testFile;
const int chipSelect = BUILTIN_SDCARD;
const char* filename = "test.log";
bool fileDone = false;









/*
* Setup
* Initialize onboard neopixel, onboard led, and serial ports
*/
void setup()
{
	//Serial.begin( 115200 );

	//// MAVLink interface start
	//Serial1.begin( 115200 );


 // 	// Inialize onboard led
	//pinMode( LEDPIN, OUTPUT );
	//digitalWrite( LEDPIN, HIGH );  // turn the LED on by making the voltage HIGH
	//
	// // Test file
	//Serial.print( "Initializing SD card..." );

	//if ( !SD.begin( chipSelect ) ) {
	//	Serial.printf( "initialization failed!\r\n" );
	//	return;
	//}
	//Serial.println( "initialization done." );

	//if ( SD.exists( filename ) ) {
	//	Serial.printf( "%s exists.\r\n", filename );
	//}
	//else {
	//	Serial.printf( "%s doesn't exist.", filename );
	//}

	//// open a new file and immediately close it:
	//Serial.printf( "Opening %s...\r\n", filename );
	//testFile = SD.open( filename, FILE_WRITE );
	//testFile.close();


}


void loop()
{

	//// Get run time in milliseconds
	//currentMillisMAVLink = millis();

	//// check if enough time has past for next MAVLink heatbeat message
	//if ( currentMillisMAVLink - previousMAVLinkMilliseconds >= nextIntervalMAVLinkMilliseconds )
	//{
	//	mavlink_message_t mavlinkMessage;

	//	// Store the current time for next loop
	//	previousMAVLinkMilliseconds = currentMillisMAVLink;

	//	// Pack the MAVLink heartbeat message
	//	mavlink_msg_heartbeat_pack( sysid, compid, &mavlinkMessage, type, autopilot_type, system_mode, custom_mode, system_state );

	//	// Copy the message to the send buffer
	//	len = mavlink_msg_to_send_buffer( buffer, &mavlinkMessage );

	//	// Write buffer containing heartbeat message
	//	Serial1.write( buffer, len );

	//	heartbeatCount++;
	//	if ( heartbeatCount >= numberOfHeartBeatsToWait )
	//	{
	//		// Request streams from Pixhawk

	//		requestMavLinkData();
	//		heartbeatCount = 0;

	//	}

	//}


	//receiveMavLinkData2( readByteFromFile );
}


//
//void requestMavLinkData()
//{
//
//	/*
//	 * Definitions are in common.h: enum MAV_DATA_STREAM
//	 *
//	 * MAV_DATA_STREAM_ALL=0, // Enable all data streams
//	 * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
//	 * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
//	 * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
//	 * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
//	 * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
//	 * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
//	 * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
//	 * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
//	 * MAV_DATA_STREAM_ENUM_END=13,
//	 *
//	 * Data in PixHawk available in:
//	 *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
//	 *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
//	 */
//
//	 // To be setup according to the needed information to be requested from the flight controller
//	const int maxStreams = 1;
//	const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_ALL };
//	const uint16_t MAVRates[maxStreams] = { 0x02 };
//	mavlink_message_t mavlinkMessage;
//
//	for ( int i = 0; i < maxStreams; i++ )
//	{
//		mavlink_msg_request_data_stream_pack( 2, 200, &mavlinkMessage, 1, 0, MAVStreams[i], MAVRates[i], 1 );
//		len = mavlink_msg_to_send_buffer( buffer, &mavlinkMessage );
//		Serial1.write( buffer, len );
//	}
//}
//
//
//bool readByteFromFile( uint8_t* buffer ) {
//	return testFile.readBytes( buffer, 1 ) == 1;
//}
//
//bool readByteFromSerial( uint8_t* buffer ) {
//
//	if ( Serial1.available() > 0 )
//	{
//		return Serial1.readBytes( buffer, 1 ) == 1;
//	}
//
//	return false;
//}
//
//void receiveMavLinkData2( bool readByte( uint8_t* ) )
//{
//	ROVER_MODE roverMode = ROVER_MODE_INITIALIZING;
//
//	if ( !fileDone ) {
//		testFile = SD.open( filename, FILE_READ );
//		int loopCount = 0;
//
//
//		while ( !fileDone )
//		{
//
//			uint8_t byteBuffer;
//
//			while ( readByte( &byteBuffer ) )
//			{
//
//				digitalWrite( LEDPIN, HIGH ); // turn the LED on (HIGH is the voltage level)
//				mavlink_message_t mavlinkMessage;
//				mavlink_status_t status;
//
//
//
//				// Try to get a new message
//				if ( mavlink_parse_char( MAVLINK_COMM_1, byteBuffer, &mavlinkMessage, &status ) == MAVLINK_FRAMING_OK )
//				{
//					loopCount += 1;
//					//Serial.printf("%d Got message id: #%d\r\n", loopCount, mavlinkMessage.msgid);
//
//
//					// Handle message
//					switch ( mavlinkMessage.msgid )
//					{
//						case MAVLINK_MSG_ID_HEARTBEAT: // #0: Heartbeat
//							{
//
//								mavlink_heartbeat_t heartbeat;
//								mavlink_msg_heartbeat_decode( &mavlinkMessage, &heartbeat );
//
//								if ( heartbeat.type == (uint8_t)MAV_TYPE_GROUND_ROVER )
//									if ( heartbeat.custom_mode != (uint32_t)roverMode ) {
//										Serial.printf( "Rover mode changed from %d to %d\r\n", roverMode, heartbeat.custom_mode );
//										roverMode = (ROVER_MODE)heartbeat.custom_mode;
//									}
//							}
//							break;
//
//						case MAVLINK_MSG_ID_SYS_STATUS: // #1: SYS_STATUS
//							{
//								mavlink_sys_status_t sys_status;
//								mavlink_msg_sys_status_decode( &mavlinkMessage, &sys_status );
//								//Serial.println("Got system status from MAVLink");
//							}
//							break;
//
//						case MAVLINK_MSG_ID_PARAM_VALUE: // #22: PARAM_VALUE
//							{
//								mavlink_param_value_t param_value;
//								mavlink_msg_param_value_decode( &mavlinkMessage, &param_value );
//								// Serial.println("Got parameter data from MAVLink");
//							}
//							break;
//
//						case MAVLINK_MSG_ID_RAW_IMU: // #27: RAW_IMU
//							{
//								mavlink_raw_imu_t imuRaw;
//								mavlink_msg_raw_imu_decode( &mavlinkMessage, &imuRaw );
//								//Serial.printf("IMU y gyro is %d\r\n", raw_imu.ygyro);
//							}
//							break;
//
//							case MAVLINK_MSG_ID_GPS_RAW_INT: // 24
//							{
//							    mavlink_gps_raw_int_t gpsRaw;
//							    mavlink_msg_gps_raw_int_decode(&mavlinkMessage, &gpsRaw);
//							    Serial.printf("GPS %d fix type is %d\r\n", mavlinkMessage.compid, gpsRaw.fix_type);
//
//							}
//							break;
//
//						case MAVLINK_MSG_ID_GPS_INPUT: // 232
//							{
//								mavlink_gps_input_t gpsInput;
//								mavlink_msg_gps_input_decode( &mavlinkMessage, &gpsInput );
//								Serial.printf( "GPS %d fix type is %d\r\n", gpsInput.gps_id, gpsInput.fix_type );
//
//							}
//							break;
//
//
//						case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: // #62
//							{
//								mavlink_nav_controller_output_t navOutput;
//								mavlink_msg_nav_controller_output_decode( &mavlinkMessage, &navOutput );
//								Serial.printf( "Distance to waypont is %d\r\n", navOutput.wp_dist );
//							}
//							break;
//
//						case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
//							{
//								mavlink_mission_item_reached_t itemReached;
//								mavlink_msg_mission_item_reached_decode( &mavlinkMessage, &itemReached );
//								Serial.printf( "Destination reached: %d\r\n", itemReached.seq );
//
//							}
//							break;
//
//						default:
//							break;
//
//					}
//
//				}
//
//
//			}
//
//			fileDone = true;
//		}
//
//		testFile.close();
//
//		digitalWrite( LEDPIN, LOW );  // turn the LED off by making the voltage LOW
//	}
//
//}
