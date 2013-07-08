#include "au_uav_ros/collision_avoidance.h"

void au_uav_ros::CollisionAvoidance::init()	{


}

au_uav_ros::Command au_uav_ros::CollisionAvoidance::avoid(au_uav_ros::Telemetry telem)	{

}

void au_uav_ros::CollisionAvoidance::setGoalWaypoint(au_uav_ros::Command com)	{
	goal_wp = com;
}
