#ifndef XBEE_TALKER_H
#define XBEE_TALKER_H


#include <errno.h>	
#include <boost/thread.hpp>
#include "au_uav_ros/serial_talker.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

//mavlink stuff
#include "au_uav_ros/mavlink_read.h"
#include "mavlink/v1.0/ardupilotmega/mavlink.h"
#include <au_uav_ros/Telemetry.h>
/*
 * Possible bug. Unable to contact roscore, need to put in provision?
 *
 * What to do if - can't open port
 * 		 - can open port, but need to close? right now only closes on receiving a 'q'
 * 		 
 */


namespace au_uav_ros{
	class XbeeTalker	{
	private:
		SerialTalker m_xbee;			
		std::string m_port;
		int m_baud;

		//mavlink stuff
		int sysid;
		int compid;	//i have no idea what these do
		int serial_compid;
		bool pc2serial;	
		int updateIndex;
		int WPSendSeqNum;

		//ros stuff
		ros::NodeHandle m_node;
		ros::Publisher m_telem_pub;
		ros::Subscriber telem_sub;	//Subscribes to my telemetry msgs
	public:
		XbeeTalker();
		XbeeTalker(std::string port, int baud);
		
		bool init(ros::NodeHandle _n);	//Opens  and sets up port, sets up ros stuff .
		void run();
		void shutdown();
	
		//In - reading from xbee/listening
		void listen();

		//Out - writing to xbee
		void spinThread();				//spin() and listens for myTelemCallbacks
//		void myTelemCallback(std_msgs::String msg);	//broadcasts my telem messages
		void myTelemCallback(au_uav_ros::Telemetry tUpdate);	//broadcasts my telem messages

		bool convertMavlinkTelemetryToROS(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate); 
		bool convertROSToMavlinkTelemetry(au_uav_ros::Telemetry &tUpdate, mavlink_au_uav_t &mavMessage); 
	};
}
#endif
