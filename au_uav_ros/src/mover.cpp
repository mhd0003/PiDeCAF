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

	//State changing will be done in this function, instead of in move(). This GCS callback will be executed not as frequently as the move,
	//since not many gcs commands will be coming in.

	//TESTING STUFF - Quick Emergency Protocol - START and STOP publishing to ca_commands to prevent overtaking manual mode.
	if(planeID == com.planeID)	{
		enum state temp;
		if(com.latitude == EMERGENCY_PROTOCOL_LAT)	{
			int incomingCommand = (int)com.longitude;
			switch(incomingCommand)	{
			case(META_START_CA_ON_LON):	
				fprintf(stderr, "Mover::CHANGING TO GREEN CA OFF MODE\n");
				//change state to using CA 
				state_change_lock.lock();
				current_state = ST_GREEN_CA_ON;
				state_change_lock.unlock();
				break;
			//No matter the state, STOP publishing.
			case(META_STOP_LON):	
				fprintf(stderr, "Mover::CHANGING TO NOGO MODE\n");
				//change state to NOGO
				state_change_lock.lock();
				current_state = ST_RED;
				state_change_lock.unlock();
				break;
			case(META_START_CA_OFF_LON):
				fprintf(stderr, "Mover::CHANGING TO GREEN CA ON MODE\n");
				//change state to not using CA 
				state_change_lock.lock();
				current_state = ST_GREEN_CA_OFF;
				state_change_lock.unlock();
				break;
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
}

//node functions
//----------------------------------------------------

bool au_uav_ros::Mover::init(ros::NodeHandle n, bool real_id)	{
	//Ros stuff
	nh = n;

	IDclient = nh.serviceClient<au_uav_ros::planeIDGetter>("getPlaneID");
	ca_commands = nh.advertise<au_uav_ros::Command>("ca_commands", 10);	
	all_telem = nh.subscribe("all_telemetry", 10, &Mover::all_telem_callback, this);
	gcs_commands = nh.subscribe("gcs_commands", 20, &Mover::gcs_command_callback, this);	
	

	//Find out my Plane ID.
	//-> need timed call? or keep trying to get plane id???
	if(!real_id)
		planeID = 999;
	else	{
		au_uav_ros::planeIDGetter srv;
		if(IDclient.call(srv))	{
			ROS_INFO("mover::init Got plane ID %d", srv.response.planeID);
			ROS_INFO("mover::init Got initial position lat: %f|long: %f|alt: %f",
					srv.response.initialLatitude, srv.response.initialLongitude, srv.response.initialAltitude);
			planeID = srv.response.planeID;
			goal_wp.planeID = planeID;
			goal_wp.latitude = initialLat = srv.response.initialLatitude;
			goal_wp.longitude= initialLong = srv.response.initialLongitude;
			goal_wp.latitude = initialAlt = srv.response.initialLatitude;
		}
		else	{
			ROS_ERROR("mover::init Unsuccessful get plane ID call.");//what to do here.... keep trying?
			return false;
		}
	}
	//CA init
	ca.init(planeID);

	current_state = ST_RED;
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
	
	while(ros::ok())	{
		//state machine fun
		//note - current state is changed in gcs_callback
		//First, get the current state
		enum state temp;
		state_change_lock.lock();
		temp = current_state;
		state_change_lock.unlock();
		//then do some switching
		switch(current_state)	{
			case(ST_RED):
				//DO NOT PUBLISH! DO NOT PUBLISH!
				break;
			case(ST_GREEN_CA_OFF):
				ros::Duration(0.25).sleep(); 	//no swamping the ardupilot, it's a delicate thing 
				goalCommandPublish();
				break;
			case(ST_GREEN_CA_ON):
				ros::Duration(0.25).sleep(); 	//no swamping the ardupilot, it's a delicate thing 
				caCommandPublish();
				break;
		}
		
	}
}

//Assumes we are in ST_GREEN_CA_OFF
void au_uav_ros::Mover::goalCommandPublish()	{
	fprintf(stderr, "mover::(ST_GREEN_CA_OFF) PUBLISHING goal COMMAND!\n");
	au_uav_ros::Command com;

	//Just send out goal wp, no collision avoidance.
	goal_wp_lock.lock();
	com = goal_wp;	
	goal_wp_lock.unlock();
	ca_commands.publish(com);

}

//Assumes we are in ST_GREEN_CA_ON mode.
void au_uav_ros::Mover::caCommandPublish()	{
	fprintf(stderr, "mover::(ST_GREEN_CA_ON) PUBLISHING CA COMMAND!\n");
	au_uav_ros::Command com;
	
	bool empty_ca_q = false;
	ca_wp_lock.lock();
	empty_ca_q = ca_wp.empty();
	if(!empty_ca_q)	{
		com = ca_wp.front();
		ca_wp.pop_front();	
	}
	ca_wp_lock.unlock();	

	//don't want to forward deafult command, if no command is returned
//	if(com.latitude != INVALID_GPS_COOR && com.latitude !=0)
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
	//n.param("fake_id", fake , false); //not very successfull. set aside for later.
	bool use_real_id = true;	
	au_uav_ros::Mover mv;
//	if(mv.init(n, use_real_id))
	if(mv.init(n, false))	//channnge if doing real planes to true
		mv.run();	
	//spin and do move logic in separate thread

}


