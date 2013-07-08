#ifndef AU_MAVLINK_READ_H
#define AU_MAVLINK_READ_H
/*
 * library for mavlink read, specific to our au_uav_ros stuff
 * wheeee
 */

#include <au_uav_ros/Telemetry.h>
#include <au_uav_ros/Command.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "au_uav_ros/serial_talker.h"
#include "mavlink/v1.0/common/mavlink.h"
//#include "mavlink/v1.0/ardupilotmega/mavlink.h"

namespace au_uav_ros	{
	namespace mav	{


		//Description:
		//	This function will take a mavlink telemetry message and convert it so that a valid
		//	ROS telemetry update message is created
		//Usage:
		//	Converting mavlink messages received in the ardu node so that they can be used in the
		//	collision avoidance node
		bool convertMavlinkTelemetryToROS(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate); 

		//Description:
		//	This function will take a mavlink telemetry message and then set tUpdate to have the same raw,
		//	unconverted fields. This raw data will be used to create a new mavlink message to send
		//	at a later point in time
		//Usage:
		//	Passing telemetry information received from ardu node to the xbee node so that the telemetry
		//	information of the running plane can be passed to other planes
		bool rawMavlinkTelemetryToRawROSTelemetry(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate);



		//Description:
		//	This function will take a mavlink command message and then set cmdToForward to have the
		//	correct data for sending a new waypoint command
		//Usage:
		//	Used to take a mavlink message and then convert it to a ROS message so that it can be passed
		//	to the collision avoidance node
		bool convertMavlinkCommandToROS(mavlink_mission_item_t &receivedCommand, au_uav_ros::Command &cmdToForward);



		//Description:
		//	This function will listen in on the serial line provided by SerialTalker and will continue to
		//	do so until a mavlink message can be decoded. Once a message is decoded, it will be returned
		//Usage:
		//	Use to obtain a command/telemetry update from a serial line
		mavlink_message_t readMavlinkFromSerial(SerialTalker &serialIn);
	
	}//end mav
}//end au_uav_ros

#endif
