#include "au_uav_ros/mover.h"

//callbacks
//----------------------------------------------------
void au_uav_ros::Mover::all_telem_callback(au_uav_ros::Telemetry telem)	{

	//CA will go here.
	//It's OK to have movement/publishing ca-commands here, since this will be called
	//when ardupilot publishes *my* telemetry msgs too.

	//lock
	if(telem.planeID == planeID)
		myTelem = telem;
	//unlock

	au_uav_ros::Command com = ca.avoid(telem);	

	//Check if ca_waypoint should be ignored
	if(com.latitude == INVALID_GPS_COOR && com.longitude == INVALID_GPS_COOR && com.altitude == INVALID_GPS_COOR)	{
		//ignore.
	}		
	else	{
		//Check if collision avoidance waypoint should be queued up, or replace all previous ca waypoints.	
		//no more non replace queue up
		ca_wp_lock.lock();
		ca_wp_sent = false;
		ca_wp.clear();
		ca_wp.push_back(com);	
		ca_wp_lock.unlock();	
		
		
	}
}

void au_uav_ros::Mover::gcs_command_callback(au_uav_ros::Command com)	{
	//Add this to the goal_wp q.
	goal_wp_lock.lock();
	goal_wp = com;	
	goal_wp_lock.unlock();
	ROS_INFO("Received new command with lat%f|lon%f|alt%f", com.latitude, com.longitude, com.altitude);
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
		ros::Duration(0.25).sleep();	//don't swamp sending of commands
		
		bool empty_ca_q = false;	//Is the CA queue empty?
		bool should_publish = false;	//Has the latest CA been published already?
		
		//Is CA_q not empty?
		ca_wp_lock.lock();
		empty_ca_q = ca_wp.empty();
		if(!empty_ca_q)	{
			if(!ca_wp_sent)	{	//It's a brand new CA wp.
				com = ca_wp.front();
				should_publish = true;
				ca_wp_sent = true;
			}
			else	{		//It's already been published.
				//Have I reached it? 	
				if( myTelem.distanceToDestination <  WP_REACH_DISTANCE &&
					myTelem.destLatitude == com.latitude &&
					myTelem.destLongitude == com.longitude &&
					myTelem.destAltitude == com.altitude)	{
					ca_wp_sent = false;
					ca_wp.pop_front();	
				}
			}
		}
		ca_wp_lock.unlock();	

		//If collision avoidance is empty, send goal_wp
		if(empty_ca_q)	{
			should_publish = true;
			com = goal_wp;	
		}	

		if(should_publish)	{
			ca_commands.publish(com);
		}

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


