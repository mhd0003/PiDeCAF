#include "au_uav_ros/mover.h"

//callbacks
void au_uav_ros::Mover::telem_callback(au_uav_ros::Telemetry telem)	{

	//CA will go here.
	//It's OK to have movement/publishing ca-commands here, since this will be called
	//when ardupilot publishes *my* telemetry msgs too.
	
	
	

}

void au_uav_ros::Mover::gcs_command_callback(au_uav_ros::Command com)	{
	//Add this to the goal_wp q.
	goal_wp_lock.lock();
	goal_wp.push(com);	
	goal_wp_lock.unlock();
}

void au_uav_ros::Mover::init(ros::NodeHandle n)	{
	nh = n;

	//ros publishers and subscribers
	ca_commands = nh.advertise<au_uav_ros::Command>("ca_commands", 10);	
	all_telem = nh.subscribe("all_telemetry", 20, &Mover::telem_callback, this);	
	gcs_commands = nh.subscribe("gcs_commands", 20, &Mover::gcs_command_callback, this);	
}








int main(int argc, char **argv)	{
	ros::init(argc, argv, "ca_logic");
	ros::NodeHandle n;
	au_uav_ros::Mover mv;
	mv.init(n);
}
