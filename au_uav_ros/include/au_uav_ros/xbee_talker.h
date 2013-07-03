#ifndef XBEE_TALKER_H
#define XBEE_TALKER_H

#include "au_uav_ros/serial_talker.h"
#include "ros/ros.h"

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
		/*
		 * init
		 * Opens  and sets up port, sets up ros stuff .
		 */
		bool init(ros::NodeHandle _n);
		void run();
		void shutdown();
	};
}
#endif
