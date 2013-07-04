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
	telem_sub = m_node.subscribe("my_telemetry", 10, &XbeeTalker::myTelemCallback, this); 

}

void au_uav_ros::XbeeTalker::run()	{
	ROS_INFO("Entering Run");

	//Spin up thread to execute myTelemCallback()
	boost::thread broadcastMyTelem(boost::bind(&XbeeTalker::spinThread, this));	

	//Start listening for telemetry and commands, upon receiving proper command
	//from ground station, execute shutdown and join

	/*Listening Time! - still has bugs :( */
	//Question: Why is it constantly spitting out INFO msgs in the listen loop?
	bool listen = true;
	char buffer[256];
	memset(buffer, '\0', 256);
	while(listen)	{
		if(read(m_xbee.getFD(), buffer, 256) == -1)	{
			ROS_INFO("error in reading");	
		}	
		if(buffer[0] == 'q')
			listen = false;
		printf("%s", buffer);
		memset(buffer, '\0', 256);
	}

	//Shutdown this node.
	ros::shutdown();
	broadcastMyTelem.join();	
}

void au_uav_ros::XbeeTalker::shutdown()	{
	//ROS_INFO("Shutting down Xbee port %s", m_port.c_str());
	printf("XbeeTalker::Shutting down Xbee port %s", m_port.c_str()); //i ros::shutdown when exiting run
	m_xbee.close_port();
}

//callbacks
void au_uav_ros::XbeeTalker::myTelemCallback(std_msgs::String msg)	{
	ROS_INFO("ding! \n");
	ROS_INFO("I got: %s\n", msg.data.c_str());

}

//spinner
void au_uav_ros::XbeeTalker::spinThread()	{
	ROS_INFO("XbeeTalker::spinThread::Starting spinner thread");
	//Handle myTelemCallback()
	ros::spin();	
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
