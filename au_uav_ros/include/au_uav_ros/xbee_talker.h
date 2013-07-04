#ifndef XBEE_TALKER_H
#define XBEE_TALKER_H


#include <boost/thread.hpp>
#include "au_uav_ros/serial_talker.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

/*
 * Possible bug. Unable to contact roscore, need to put in provision?
 *
 *now- seems to not display my reads until i start listening for callbacks?? 
 *
 * What to do if - can't open port
 * 		 - can open port, but need to close? right now only closes on receiving a 'q'
 */


namespace au_uav_ros{
	class XbeeTalker	{
	private:
		SerialTalker m_xbee;			
		std::string m_port;
		int m_baud;

		//ros stuff
		ros::NodeHandle m_node;
		ros::Subscriber telem_sub;	//Subscribes to my telemetry msgs
	public:
		XbeeTalker();
		XbeeTalker(std::string port, int baud);
		
		bool init(ros::NodeHandle _n);	//Opens  and sets up port, sets up ros stuff .
		void run();
		void shutdown();
		
		//calback - broadcast my telem message
		void myTelemCallback(std_msgs::String msg);	

		//thread to handle callbacks
		void spinThread();
	};
}
#endif
