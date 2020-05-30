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
