#include "au_uav_ros/collision_avoidance.h"

void au_uav_ros::CollisionAvoidance::init()	{

	ROS_INFO("CollisionAvoidance:init()");
}

au_uav_ros::Command au_uav_ros::CollisionAvoidance::avoid(au_uav_ros::Telemetry telem)	{
	ROS_INFO("CollisionAvoidance::avoid()");
}

void au_uav_ros::CollisionAvoidance::setGoalWaypoint(au_uav_ros::Command com)	{
	ROS_INFO("CollisionAvoidance::setGoalWaypoint()");
	goal_wp = com;

}
