// 
// 
// 

#include "EnumHelper.h"

const char* EnumHelper::convert( ROVER_MODE roverMode )
{
	switch ( roverMode )
	{

		case ROVER_MODE_MANUAL:
			return "Manual";
			break;
		case ROVER_MODE_ACRO:
			return "Acro";
			break;
		case ROVER_MODE_STEERING:
			return "Steering";
			break;
		case ROVER_MODE_HOLD:
			return "Hold";
			break;
		case ROVER_MODE_LOITER:
			return "Loiter";
			break;
		case ROVER_MODE_AUTO:
			return "Auto";
			break;
		case ROVER_MODE_RTL:
			return "Return to land";
			break;
		case ROVER_MODE_SMART_RTL:
			return "Smart return to land";
			break;
		case ROVER_MODE_GUIDED:
			return "Guided";
			break;
		case ROVER_MODE_INITIALIZING:
			return "Initialization";
			break;
		case ROVER_MODE_ENUM_END:
			return "End";
			break;

		default:
			return "Unknown";
	}
}

const char* EnumHelper::convert( MAV_MODE_FLAG mavModeFag )
{
	switch ( mavModeFag )
	{

		case MAV_MODE_FLAG_SAFETY_ARMED:
			return "Safety Armed";
			break;
		case MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
			return "Manual Input Enabled";
			break;
		case MAV_MODE_FLAG_HIL_ENABLED:
			return "HIL Enabled";
			break;
		case MAV_MODE_FLAG_STABILIZE_ENABLED:
			return "Stabilize Enabled";
			break;
		case MAV_MODE_FLAG_GUIDED_ENABLED:
			return "Guided Enabled";
			break;
		case MAV_MODE_FLAG_AUTO_ENABLED:
			return "Auto Enabled";
			break;
		case MAV_MODE_FLAG_TEST_ENABLED:
			return "Test Enabled";
			break;
		case MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
			return "Custom Mode Enabled";
			break;

		default:
			return "Unknown";
	}
}


