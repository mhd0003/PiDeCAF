#ifndef MOVER_H 
#define MOVER_H 

/*
 * Mover node
 * Team 1 2013 REU
 *
 * Responsible for running collision avoidance and generating delicious new commands for the ardupilot.
 *
 * Subscribes to:
 * 	gcs_commands - Goal waypoint commands from ground control station.
 * 	all_telemetry - All telemetry messages from other planes and myself.
 *
 * Publishes to:
 * 	ca_commands - New collision avoidance waypoint commands for Ardupilot.
 */


#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

//collision avoidance library
#include "au_uav_ros/collision_avoidance.h"
#include "au_uav_ros/planeObject.h"

#include "au_uav_ros/pi_standard_defs.h"
#include "au_uav_ros/Fsquared.h"

//ros stuff
#include "ros/ros.h"
#include "au_uav_ros/Command.h"		
#include "au_uav_ros/Telemetry.h"
#include "au_uav_ros/planeIDGetter.h"	//Srv header that returns plane ID.


namespace au_uav_ros	{
	class Mover {
		private:
			CollisionAvoidance ca;				//Main Collision Avoidance logic here.

			int planeID;					//current plane id

			//Queues for Waypoints
			au_uav_ros::Command goal_wp;			//store goal wp from Ground control 
			std::deque<au_uav_ros::Command> next_wp;	//store waypoint to go to next, produced by collision avoidance 

			//Locks for Queues
			boost::mutex goal_wp_lock;
			boost::mutex next_wp_lock;

			//ROS stuff
			ros::NodeHandle nh;
			ros::ServiceClient IDclient;	//Get my plane's ID form ardupilot node.
			ros::Publisher ca_commands;	//Publish actual CA command waypoints
			ros::Subscriber my_telem_sub;	//Subscribe to just me telemetry (in raw mav format)
			ros::Subscriber all_telem;	//Subscribe to all telemetry msgs (me and other planes)
			ros::Subscriber gcs_commands;	//Subscribe to commands from Ground control

			/*
			 * Callback for any incoming telemetry msg (including my own).
			 * Updates my position if it's my telemetry.
			 * Calls CollisionAvoidance's avoid() function.
			 * Replaces or queues up avoid()'s returned command.
			 */
			void all_telem_callback(au_uav_ros::Telemetry telem);

			
			/*
			 * callback for any ground control commands.
			 * Replaces current goal wp with incoming command.
			 */ 
			void gcs_command_callback(au_uav_ros::Command com);

			//thread stuff, calls ros::spin()
			void spinThread();

			//main decision making logic
			void move();
		public:
			int getPlaneID() {return planeID;} 
			bool init(ros::NodeHandle n);
			void run();

	};
}	


#endif	//COLLISION_AVOIDANCE_LOGIC_H
