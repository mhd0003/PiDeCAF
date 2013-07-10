#include "au_uav_ros/collision_avoidance.h"

void au_uav_ros::CollisionAvoidance::init(int planeID)	{

	ROS_INFO("CollisionAvoidance:init()");

	//Hardcode id for now
	me.setID(255);

}

//TODO: Add some method to not resend commands when the waypoint has not changed?
au_uav_ros::Command au_uav_ros::CollisionAvoidance::avoid(au_uav_ros::Telemetry telem)	{
	ROS_INFO("CollisionAvoidance::avoid() me position: %f, %f, %f | me destination: %f, %f", me.getCurrentLoc().latitude, me.getCurrentLoc().longitude, me.getCurrentLoc().altitude,
											me.getDestination().latitude, me.getDestination().longitude);
	au_uav_ros::Command newCmd;

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
}

void au_uav_ros::CollisionAvoidance::setGoalWaypoint(au_uav_ros::Command com)	{
	ROS_INFO("CollisionAvoidance::setGoalWaypoint()");
	goal_wp = com;
	au_uav_ros::waypoint dest;
	dest.latitude = com.latitude;
	dest.longitude = com.longitude;
	dest.altitude = com.altitude;
	me.setDestination(dest);
}
