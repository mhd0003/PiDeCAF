#include "au_uav_ros/mavlink_read.h"
/*
bool au_uav_ros::ArduTalker::convertROSToMavlinkTelemetry(au_uav_ros::Telemetry &tUpdate, mavlink_au_uav_t &mavMessage)	{
	
}
*/
bool au_uav_ros::mav::convertMavlinkTelemetryToROS(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate) {
	tUpdate.currentLatitude = (mavMessage.au_lat) / 10000000.0;	  /// Lattitude * 10**7 so have to divide by 10^7
	tUpdate.currentLongitude = (mavMessage.au_lng) / 10000000.0;	  /// Longitude * 10**7 so have to divide by 10^7
	tUpdate.currentAltitude = (mavMessage.au_alt) / 100.0; 	  /// Altitude in cm so divide by 100 to get meters	
	
	tUpdate.destLatitude = (mavMessage.au_target_lat) / 10000000.0;  /// Lattitude * 10**7 so have to divide by 10^7
	tUpdate.destLongitude = (mavMessage.au_target_lng) / 10000000.0; /// Longitude * 10**7 so have to divide by 10^7
	tUpdate.destAltitude = (mavMessage.au_target_alt) / 100.0; 	  /// Altitude in cm so divide by 100 to get meters

	tUpdate.groundSpeed = (mavMessage.au_ground_speed) / 100.0; /// Originally in cm / sec so have to convert to mph

	tUpdate.distanceToDestination = mavMessage.au_distance; 	  /// Distance between plane and next waypoint in meters.

	tUpdate.targetBearing = mavMessage.au_target_bearing / 100;	  /// This is the direction to the next waypoint or loiter center in degrees

	tUpdate.currentWaypointIndex = mavMessage.au_target_wp_index;   /// The current waypoint index

//	tUpdate.telemetryHeader.seq = ++updateIndex; //what is update index used for>???
	tUpdate.telemetryHeader.stamp = ros::Time::now();
	return true;
}

bool au_uav_ros::mav::rawMavlinkTelemetryToRawROSTelemetry(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate) {
        tUpdate.currentLatitude = (mavMessage.au_lat);       /// Lattitude * 10**7 so have to divide by 10^7
        tUpdate.currentLongitude = (mavMessage.au_lng);      /// Longitude * 10**7 so have to divide by 10^7
        tUpdate.currentAltitude = (mavMessage.au_alt);    /// Altitude in cm so divide by 100 to get meters     

        tUpdate.destLatitude = (mavMessage.au_target_lat);  /// Lattitude * 10**7 so have to divide by 10^7
        tUpdate.destLongitude = (mavMessage.au_target_lng); /// Longitude * 10**7 so have to divide by 10^7
        tUpdate.destAltitude = (mavMessage.au_target_alt);        /// Altitude in cm so divide by 100 to get meters

        tUpdate.groundSpeed = (mavMessage.au_ground_speed); /// Originally in cm / sec so have to convert to mph

        tUpdate.distanceToDestination = mavMessage.au_distance;           /// Distance between plane and next waypoint in me$

        tUpdate.targetBearing = mavMessage.au_target_bearing;       /// This is the direction to the next waypoint or $

        tUpdate.currentWaypointIndex = mavMessage.au_target_wp_index;   /// The current waypoint index

//      tUpdate.telemetryHeader.seq = ++updateIndex; //what is update index used for>???
        tUpdate.telemetryHeader.stamp = ros::Time::now();
        return true;
}


bool au_uav_ros::mav::convertMavlinkCommandToROS(mavlink_mission_item_t receivedCommand, au_uav_ros::Command cmdToForward){
	//If we are receiveing a command for real planes, it better not be intended for simulated planes
	cmdToForward.sim = false;
	cmdToForward.commandID = receivedCommand.command;
	//What is this for? It is not used when we are converting back to a mavlink message
	cmdToForward.param = 0;
	cmdToForward.latitude = receivedCommand.x;
	cmdToForward.longitude = receivedCommand.y;
	cmdToForward.altitude = receivedCommand.z;
	cmdToForward.commandHeader.stamp = ros::Time::now();
}




mavlink_message_t au_uav_ros::mav::readMavlinkFromSerial(SerialTalker &serialIn){
	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;

	// Blocking wait for new data
		//if (debug) printf("Checking for new data on serial port\n");
		// Block until data is available, read only one byte to be able to continue immediately
		//char buf[MAVLINK_MAX_PACKET_LEN];
	while(1){
		uint8_t cp;
		mavlink_message_t message;
		message.msgid = 5; //to ensure that we are actually receiving heartbeats
		mavlink_status_t status;
		uint8_t msgReceived = false;
		serialIn.lock();
		int num = read(serialIn.getFD(), &cp, 1);
		serialIn.unlock();
		//ROS_INFO("Read %c", (char)cp);
		if (num > 0)
		{
			// Check if a message could be decoded, return the message in case yes
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
		}

		else
		{
			//if (!silent) fprintf(stderr, "ERROR: Could not read from port %s\n", port.c_str());
		}

		// If a message could be decoded, return it
		if(msgReceived)
			return message;
	}
}


