#include "au_uav_ros/mover.h"

//callbacks
void au_uav_ros::Mover::telem_callback(au_uav_ros::Telemetry telem)	{

	//CA will go here.
	//It's OK to have movement/publishing ca-commands here, since this will be called
	//when ardupilot publishes *my* telemetry msgs too.
	
	au_uav_ros::Command com = ca.avoid(telem);	

	//Check if ca_waypoint should be ignored
	if(com.latitude == INVALID_GPS_COOR && com.longitude == INVALID_GPS_COOR && com.altitude == INVALID_GPS_COOR)	{
		//ignore.
	}		
	else	{
		//Check if collision avoidance waypoint should be queued up, or replace all previous ca waypoints.	
		if(!com.replace)	{
			ca_wp_lock.lock();
			ca_wp.push_back(com);	
			ca_wp_lock.unlock();	
		}
		else	{
			ca_wp_lock.lock();
			ca_wp.clear();
			ca_wp.push_back(com);	
			ca_wp_lock.unlock();	
		
		}
	}
}

void au_uav_ros::Mover::gcs_command_callback(au_uav_ros::Command com)	{
	//Add this to the goal_wp q.
	goal_wp_lock.lock();
	goal_wp = com;	
	goal_wp_lock.unlock();
}

void au_uav_ros::Mover::init(ros::NodeHandle n)	{
	//Ros stuff
	nh = n;

	ca_commands = nh.advertise<au_uav_ros::Command>("ca_commands", 10);	
	all_telem = nh.subscribe("all_telemetry", 20, &Mover::telem_callback, this);	
	gcs_commands = nh.subscribe("gcs_commands", 20, &Mover::gcs_command_callback, this);	

	//CA init
	ca.init();
}

void au_uav_ros::Mover::run()	{

	ROS_INFO("Mover:;Entering Run");

	//spin up thread to get callbacks
	boost::thread spinner(boost::bind(&Mover::spinThread, this));

	//Given GCS commands and ca waypoints, decide which ones to send to ardupilot	
	move();

	ros::shutdown();
	spinner.join();
}

void au_uav_ros::Mover::move()	{
}





void au_uav_ros::Mover::spinThread()	{
	ROS_INFO("mover::starting spinner thread");
	ros::spin();
}

int main(int argc, char **argv)	{
	ros::init(argc, argv, "ca_logic");
	ros::NodeHandle n;
	au_uav_ros::Mover mv;
	mv.init(n);
	mv.run();	
	//spin and do move logic in separate thread

}


