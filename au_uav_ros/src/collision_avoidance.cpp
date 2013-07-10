#include "au_uav_ros/collision_avoidance.h"

void au_uav_ros::CollisionAvoidance::init()	{

	ROS_INFO("CollisionAvoidance:init()");

}

//TODO: Add some method to not resend commands when the waypoint has not changed?
au_uav_ros::Command au_uav_ros::CollisionAvoidance::avoid(au_uav_ros::Telemetry telem)	{
	ROS_INFO("CollisionAvoidance::avoid()");
	au_uav_ros::Command newCmd;

	au_uav_ros::waypoint tempForceWaypoint = fsquared::findTempForceWaypoint(me, telem);

	//Make new command from the calculated waypoint
	//newCmd.stamp = ros::Time::now();
	//newCmd.planeId = me.getID();******************************************************************************************************************WHYYYYYYYY****************************************
	newCmd.param = 2;
	newCmd.latitude = me.getCurrentLoc().latitude;
	newCmd.longitude = me.getCurrentLoc().longitude;
	newCmd.altitude =me.getCurrentLoc().altitude;
	newCmd.replace = true;
	return newCmd;
}

void au_uav_ros::CollisionAvoidance::setGoalWaypoint(au_uav_ros::Command com)	{
	ROS_INFO("CollisionAvoidance::setGoalWaypoint()");
	goal_wp = com;

}
