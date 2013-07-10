#ifndef ARDU_TALKER_H 
#define ARDU_TALKER_H


#include <errno.h>	
#include <boost/thread.hpp>
#include <stdio.h>	//fprintf for debugging

//ros stuff
#include "au_uav_ros/serial_talker.h"
#include "au_uav_ros/mavlink_read.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

//custom msgs
#include <au_uav_ros/Telemetry.h>
#include <au_uav_ros/Command.h>
#include <au_uav_ros/planeIDGetter.h>

//mavlink stuff
#include "mavlink/v1.0/ardupilotmega/mavlink.h"
/*
 * Possible bug. Unable to contact roscore, need to put in provision?
 *
 * What to do if - can't open port
 * 		 - can open port, but need to close? right now only closes on receiving a 'q'
 * 		 
 */


namespace au_uav_ros{
	class ArduTalker	{
	private:
		SerialTalker m_ardu;			
		std::string m_port;
		int m_baud;

		//mavlink stuff
		int sysid;
		int compid;	//i have no idea what these do
		int serial_compid;
		bool pc2serial;	
		int updateIndex;
		int WPSendSeqNum;

		//plane ID stuff.
		int planeID;
		boost::mutex IDSetter;
		boost::condition_variable cond;
		bool isIDSet;
			

		//ros stuff
		ros::NodeHandle m_node;
		ros::Subscriber m_command_sub;	//Subscribes to CA commands 
		
		ros::Publisher m_telem_pub;
		ros::Publisher m_mav_telem_pub;
		ros::ServiceServer service;
	public:
		ArduTalker();
		ArduTalker(std::string port, int baud);
		
		bool init(ros::NodeHandle _n);	//Opens  and sets up port, sets up ros stuff .
		void run();
		void shutdown();
	
		//In - reading from ardu/listening
		void listen();

		//Out - writing to ardu 
		void spinThread();				//spin() and listens for myTelemCallbacks
		void commandCallback(au_uav_ros::Command cmd);	//sends commands to ardu

		//Getplane ID srv
		//Returns "me's" ID. May return -1, if id not initialized yet by ardupilot.
		bool getPlaneID(au_uav_ros::planeIDGetter::Request &req, 
				au_uav_ros::planeIDGetter::Response &res); 
	};
}
#endif
