#include "au_uav_ros/serial_talker.h"
#include <cstdio> //printf
#include <string>
#include <ros/console.h>	//used for debugging
int main()	{
	
	std::string port = "/dev/ttyUSB0";
	SerialTalker xbee;

	if(xbee.open_port(port) == -1)	{
		ROS_INFO("Could not open port %s ", port.c_str());	
	}
	else	{
		ROS_INFO("Successfully opened port %s", port.c_str());
		std::cout << "HEllo world!! " <<std::endl;
		xbee.setup_port(9600, 8, 1, true); 

		
		xbee.close_port();
	}


}
