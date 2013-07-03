#include "au_uav_ros/xbee_talker.h"
#include <cstdio> //printf
#include <string>
#include <ros/console.h>	//used for debugging

au_uav_ros::XbeeTalker::XbeeTalker()	{
	m_port = "dev/ttyUSB0";
	m_baud = 9600;
}

au_uav_ros::XbeeTalker::XbeeTalker(std::string _port, int _baud)	{
	m_port = _port;
	m_baud = _baud;	
	
}

bool au_uav_ros::XbeeTalker::init(ros::NodeHandle _n)	{
	//Open and setup port.
	if(m_xbee.open_port(m_port) == -1)	{
		ROS_INFO("Could not open port %s", m_port.c_str());
	}
	else	{
		ROS_INFO("opened port %s", m_port.c_str());
		m_xbee.setup_port(m_baud, 8, 1, true);		
	}
	
	//Set up Ros stuff.
	m_node = _n;
	//dang it. what's a telemetry msg look like?
	//telem_sub = m_node.subscribe("my_telemetry", 10, my_telem_callback); 

}

void au_uav_ros::XbeeTalker::run()	{

	
}

void au_uav_ros::XbeeTalker::shutdown()	{
	ROS_INFO("Shutting down Xbee port %s", m_port.c_str());
	m_xbee.close_port();
}

int main(int argc, char** argv)	{

	std::cout << "helo world!" <<std::endl;
	std::string port = "/dev/ttyUSB0";

	ros::init(argc, argv, "XbeeTalker");
	ros::NodeHandle n;
	au_uav_ros::XbeeTalker talk(port, 9600);
	talk.init(n);
	talk.run();
	talk.shutdown();
}
