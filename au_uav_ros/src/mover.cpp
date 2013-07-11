#include "au_uav_ros/mover.h"

//callbacks
//----------------------------------------------------
void au_uav_ros::Mover::all_telem_callback(au_uav_ros::Telemetry telem)	{

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
		else{
			ca_wp_lock.lock();
			ca_wp.clear();
			ca_wp.push_back(com);	
			ca_wp_lock.unlock();	
		
		}
	}
}

void au_uav_ros::Mover::gcs_command_callback(au_uav_ros::Command com)	{
	
	//TESTING STUFF - Quick Emergency Protocol - START and STOP publishing to ca_commands to prevent overtaking manual mode.
	enum state temp;
	if(com.latitude == EMERGENCY_PROTOCOL_LAT)	{
		if(com.longitude == EMERGENCY_START_LON)	{
			//change state to OK
			state_change_lock.lock();
			current_state = ST_OK;
			state_change_lock.unlock();
		}
		if(com.longitude == EMERGENCY_STOP_LON)	{
			//change state to NOGO
			state_change_lock.lock();
			current_state = ST_NO_GO;
			state_change_lock.unlock();
		}
	}
	else	{
		//just a regular old gcs command, move along now.
		//Add this to the goal_wp q.
		goal_wp_lock.lock();
		goal_wp = com;	
		goal_wp_lock.unlock();
		ROS_INFO("Received new command with lat%f|lon%f|alt%f", com.latitude, com.longitude, com.altitude);
		ca.setGoalWaypoint(com);

	}	
	
}

//node functions
//----------------------------------------------------

bool au_uav_ros::Mover::init(ros::NodeHandle n, bool fake)	{
	//Ros stuff
	nh = n;

	IDclient = nh.serviceClient<au_uav_ros::planeIDGetter>("getPlaneID");
	ca_commands = nh.advertise<au_uav_ros::Command>("ca_commands", 10);	
	all_telem = nh.subscribe("all_telemetry", 20, &Mover::all_telem_callback, this);	
	gcs_commands = nh.subscribe("gcs_commands", 20, &Mover::gcs_command_callback, this);	
	

	//Find out my Plane ID.
	//-> need timed call? or keep trying to get plane id???
	if(fake)
		planeID = 999;
	else	{
		au_uav_ros::planeIDGetter srv;
		if(IDclient.call(srv))	{
			ROS_INFO("mover::init Got plane ID %d", srv.response.planeID);
			planeID = srv.response.planeID;
		}
		else	{
			ROS_ERROR("mover::init Unsuccessful get plane ID call.");//what to do here.... keep trying?
			return false;
		}
	}
	//CA init
	ca.init(planeID);

	current_state = ST_NO_GO;
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
	
	while(ros::ok()){
		//state machine fun
		//note - current state is changed in gcs_callback
		//First, get the current state
		enum state temp;
		state_change_lock.lock();
		temp = current_state;
		state_change_lock.unlock();
		//then do some switching
		switch(current_state)	{
			case(ST_NO_GO):
				//DO NOT PUBLISH! DO NOT PUBLISH!
				break;
			case(ST_OK):
				//ok publish.
				caCommandPublish();
				break;
		}
	}
}

//Assumes we are in ST_OK mode.
void au_uav_ros::Mover::caCommandPublish()	{
	au_uav_ros::Command com;
	//If collision avoidance is NOT EMPTY, that takes precedence
	bool empty_ca_q = false;
	//attempt to limit the number of commands sent to the ardupilot,
	//we can still process telemetry updates quickly, but we only send 4 commands
	//a second
	ros::Duration(0.25).sleep();
	ca_wp_lock.lock();
	empty_ca_q = ca_wp.empty();
	if(!empty_ca_q)	{
		com = ca_wp.front();
		ca_wp.pop_front();	
	}
	ca_wp_lock.unlock();	

	ca_commands.publish(com);

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
	bool fake;
	n.param("fake_id", fake, false);
	au_uav_ros::Mover mv;
	if(mv.init(n, fake))
		mv.run();	
	//spin and do move logic in separate thread

}


