#ifndef MOVER_H 
#define MOVER_H 


#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

//collision avoidance library
#include "au_uav_ros/collision_avoidance.h"

//ros stuff
#include "ros/ros.h"
#include "au_uav_ros/Command.h"
#include "au_uav_ros/Telemetry.h"


namespace au_uav_ros	{
	class Mover {
		private:
			//Collision Avoidance fun
			CollisionAvoidance ca;

			//Queues for Waypoints
			std::queue<au_uav_ros::Command> goal_wp;	//store goal wps from Ground control 
			std::queue<au_uav_ros::Command> ca_wp;	 	//store collision avoidance waypoints 

			//Locks for Queues
			boost::mutex goal_wp_lock;
			boost::mutex ca_wp_lock;

			//ROS stuff
			ros::NodeHandle nh;
			ros::Publisher ca_commands;	//Publish actual CA command waypoints
			ros::Subscriber all_telem;	//Subscribe to all telemetry msgs (me and other planes)
			ros::Subscriber gcs_commands;	//Subscribe to commands from Ground control

			void telem_callback(au_uav_ros::Telemetry telem);
			void gcs_command_callback(au_uav_ros::Command com);

			//thread stuff
			void spinThread();

			//main decision making logic
			void move();
		public:
//			CollisionAvoidanceLogic();
			void init(ros::NodeHandle n);
			void run();

	};
}	


#endif	//COLLISION_AVOIDANCE_LOGIC_H
