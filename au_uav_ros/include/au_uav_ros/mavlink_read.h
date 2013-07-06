#ifndef AU_MAVLINK_READ_H
#define AU_MAVLINK_READ_H
/*
 * library for mavlink read, specific to our au_uav_ros stuff
 * wheeee
 */
#include <au_uav_ros/Telemetry.h>
#include <au_uav_ros/Command.h>

#include "mavlink/v1.0/ardupilotmega/mavlink.h"
namespace au_uav_ros	{
	namespace mav	{
		bool convertMavlinkTelemetryToROS(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate); 
	
	
	}//end mav
}//end au_uav_ros

#endif
