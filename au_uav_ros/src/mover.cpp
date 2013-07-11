/*
 * Mover node
 * 
 * main()
 * 	init() -initializes ROS items and collision avoidance obj, obtains planeID from Ardupilot. 
 *	run() - Spins up new thread to ros::spin() for callbacks.
 *		Calls move() - Publishes next_wp at 4 hz.    
 * 		 
 * callbacks
 * 	all_telemetry 	Calls collision avoidance (ca.avoid()) and sets next_wp.	
 *	gcs_command	Sets goal_wp to newly received goal waypoint.
 *
 */


#include "au_uav_ros/mover.h"

//callbacks
//----------------------------------------------------
void au_uav_ros::Mover::all_telem_callback(au_uav_ros::Telemetry telem)	{

	//CA will go here.
	//It's OK to have movement/publishing ca-commands here, since this will be called
	//when ardupilot publishes *my* telemetry msgs too.


	au_uav_ros::Command com = ca.avoid(telem);	

	//No collision avoidance waypoint, send goal_wp instead.
	if(com.latitude == INVALID_GPS_COOR && com.longitude == INVALID_GPS_COOR && com.altitude == INVALID_GPS_COOR)	{
		goal_wp_lock.lock();
		com = goal_wp;
		goal_wp_lock.lock();
	}		
	next_wp_lock.lock();
	next_wp.clear();
	next_wp.push_back(com);	
	next_wp_lock.unlock();	
}

void au_uav_ros::Mover::gcs_command_callback(au_uav_ros::Command com)	{
	//Add this to the goal_wp q.
	goal_wp_lock.lock();
	goal_wp = com;	
	goal_wp_lock.unlock();
	ROS_INFO("Received new command with lat%f|lon%f|alt%f", com.latitude, com.longitude, com.altitude);
	
	//may need to lock this... noooooooonoooooooooo
	////TODO TODO TODO TODO
	ca.setGoalWaypoint(com);
}

//node functions
//----------------------------------------------------

bool au_uav_ros::Mover::init(ros::NodeHandle n)	{
	//Ros stuff
	nh = n;

	IDclient = nh.serviceClient<au_uav_ros::planeIDGetter>("getPlaneID");
	ca_commands = nh.advertise<au_uav_ros::Command>("ca_commands", 10);	
	all_telem = nh.subscribe("all_telemetry", 20, &Mover::all_telem_callback, this);	
	gcs_commands = nh.subscribe("gcs_commands", 20, &Mover::gcs_command_callback, this);	
	

	//Find out my Plane ID.
	//-> need timed call? or keep trying to get plane id???
	au_uav_ros::planeIDGetter srv;
	if(IDclient.call(srv))	{
		ROS_INFO("mover::init Got plane ID %d", srv.response.planeID);
		planeID = srv.response.planeID;
	}
	else	{
		ROS_ERROR("mover::init Unsuccessful get plane ID call.");//what to do here.... keep trying?
		return false;
	}
	
	//CA init
	ca.init(planeID);
	return true;
}

void au_uav_ros::Mover::run()	{

	ROS_INFO("Entering mover::run()");

	//spin up thread to get callbacks
	boost::thread spinner(boost::bind(&Mover::spinThread, this));

	//Given GCS commands and ca waypoints, decide which ones to send to ardupilot	
	move();

	ros::shutdown();
	spinner.join();
}

void au_uav_ros::Mover::move()	{

	ROS_INFO("Entering mover::move()");	
	au_uav_ros::Command com;

	//Some shutdown method.. how to??????
	while(ros::ok()){
		//attempt to limit the number of commands sent to the ardupilot,
		//we can still process telemetry updates quickly, but we only send 4 commands
		//a second
		ros::Duration(0.25).sleep();
		next_wp_lock.lock();
		if(!empty_ca_q)	{
			com = next_wp.front();
			next_wp.pop_front();	
		}
		next_wp_lock.unlock();	
		//PROBLEM: Don't want to swamp the ardupilot with too many commands. 
		//Check to see if current destination is any of these, if so, don't send don't send!
		
		ca_commands.publish(com);

	}
}


void au_uav_ros::Mover::spinThread()	{
	ROS_INFO("mover::starting spinner thread");
	ros::spin();
}
//main
//----------------------------------------------------
int main(int argc, char **argv)	{
	ros::init(argc, argv, "ca_logic");
	ros::NodeHandle n;
	au_uav_ros::Mover mv;
	if(mv.init(n))
		mv.run();	
	//spin and do move logic in separate thread

}


