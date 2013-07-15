#include "au_uav_ros/collision_avoidance.h"

void au_uav_ros::CollisionAvoidance::init(int planeID)	{

	ROS_INFO("CollisionAvoidance:init()");
	me.setID(planeID);

}

//TODO: Add some method to not resend commands when the waypoint has not changed?
au_uav_ros::Command au_uav_ros::CollisionAvoidance::avoid(au_uav_ros::Telemetry telem)	{
	bool runAPF = false;

	if (runAPF) {
		ROS_INFO("CollisionAvoidance::avoid() me position: %f, %f, %f | me destination: %f, %f", me.getCurrentLoc().latitude, me.getCurrentLoc().longitude, me.getCurrentLoc().altitude,
												me.getDestination().latitude, me.getDestination().longitude);
		au_uav_ros::Command newCmd;

		//Setting goalwp in plane object
		au_uav_ros::waypoint dest;
		goal_wp_lock.lock();
		dest.latitude = goal_wp.latitude;
		dest.longitude = goal_wp.longitude;
		dest.altitude = goal_wp.altitude;
		goal_wp_lock.unlock();
		me.setDestination(dest);

		au_uav_ros::waypoint tempForceWaypoint = fsquared::findTempForceWaypoint(me, telem);
		//Make new command from the calculated waypoint
		//newCmd.stamp = ros::Time::now();
		newCmd.planeID = me.getID();
		newCmd.commandID = 2;
		newCmd.param = 2;
		newCmd.latitude = tempForceWaypoint.latitude;
		newCmd.longitude = tempForceWaypoint.longitude;
		newCmd.altitude = me.getDestination().altitude;
		newCmd.replace = true;
		return newCmd;
	} else {
		return runIPN(telem);
	}
}

/*
 * If the update is from another plane, I update my map.
 * I return a command with INVALID_GPS_COOR
 * 
 * If the update is from me, I update my "me" planeObject.
 * I run IPN to check for threats. If I'm avoiding a threat,
 * I get a waypoint from IPN. If I'm not in danger, I send 
 * a command with my goal waypoint.
 */
au_uav_ros::Command au_uav_ros::CollisionAvoidance::runIPN(au_uav_ros::Telemetry telem) {
	using au_uav_ros::Command;
	using au_uav_ros::PlaneObject;
	using au_uav_ros::waypoint;

	// Debug statement options
	bool printMePosition = true;
	bool printTelemID = true;
	bool printGoalWP = true;
	bool printCommand = true;

	Command command;
	waypoint goalWP;

	// Get the current goal waypoint
	goal_wp_lock.lock();
	goalWP.latitude = goal_wp.latitude;
	goalWP.longitude = goal_wp.longitude;
	goalWP.altitude = goal_wp.altitude;
	goal_wp_lock.unlock();

	// Update the plane
	me.update(telem);
	me.setDestination(goalWP);



	if (printMePosition) {
		ROS_INFO("CollisionAvoidance::runIPN() me position: #%d - (%f, %f, %f)",
			me.getID(), me.getCurrentLoc().latitude, me.getCurrentLoc().longitude,
			me.getCurrentLoc().altitude);
	}
	if (printTelemID) {
		ROS_INFO("telem planeID: %d", me.getID(), telem.planeID);
	}
	if (printGoalWP) {
		ROS_INFO("goal_wp: (%f, %f, %f)",
			goalWP.latitude, goalWP.longitude,
			goalWP.altitude);
	}

	
	if (me.getID() == telem.planeID) {
		waypoint avoidanceWP;
		std::map<int, PlaneObject> allPlanes = me.getMap();

		int mapSize = allPlanes.size();
		ROS_INFO("Ooh! About to check for threats. me's map has %d planes", mapSize);

		if (ipn::checkForThreats(me, allPlanes, avoidanceWP)) {
			command.commandID = 2;
			command.latitude = avoidanceWP.latitude;
			command.longitude = avoidanceWP.longitude;
			command.altitude = goalWP.altitude;
		} else {
			command.commandID = 1;
			command.latitude = goalWP.latitude;
			command.longitude = goalWP.longitude;
			command.altitude = goalWP.altitude;
		}
	} else {
		// TODO: Check if reached waypoint
		command.commandID = 2;
		command.latitude = INVALID_GPS_COOR;
		command.longitude = INVALID_GPS_COOR;
		command.altitude = INVALID_GPS_COOR;
	}

	command.planeID = me.getID();
	command.param = 2;
	command.replace = true;

	if (printCommand) {
		ROS_INFO("Returning command: #%d - (%f, %f, %f)", command.planeID,
			command.latitude, command.longitude, command.altitude);
	}

	return command;
}

void au_uav_ros::CollisionAvoidance::setGoalWaypoint(au_uav_ros::Command com)	{
	ROS_INFO("CollisionAvoidance::setGoalWaypoint()");
	
	goal_wp_lock.lock();
	goal_wp = com;
	goal_wp_lock.unlock();
}
