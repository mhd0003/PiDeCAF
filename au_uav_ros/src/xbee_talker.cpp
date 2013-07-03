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

bool au_uav_ros::XbeeTalker::init()	{
	if(m_xbee.open_port(m_port) == -1)	{
		ROS_INFO("Could not open port %s", m_port.c_str());
	}
	else	{
		ROS_INFO("opened port %s", m_port.c_str());
//		xbee.setup_port(baud		
	}


}

void au_uav_ros::XbeeTalker::shutdown()	{
	ROS_INFO("Shutting down Xbee port %s", m_port.c_str());
	m_xbee.close_port();
}

int main()	{

	std::cout << "helo world!" <<std::endl;
	std::string port = "/dev/ttyUSB0";
	au_uav_ros::XbeeTalker talk(port, 9600);
	talk.init();
	talk.shutdown();
/*	

	bool listen = true;
	if(xbee.open_port(port) == -1)	{
	}
	else	{
		xbee.setup_port(9600, 8, 1, true); 
		
		char* msg = "Hello, I'm listening!";
		int written = write(xbee.getFD(), (char*) msg, strlen(msg));

		char buffer[256];	
		memset(buffer, '\0', 256);
		while(listen)	{
			printf("Blocking, waiting for read");
			if(read(xbee.getFD(), buffer, 256) == -1)	{
				printf("error in reading errno %d", strerror(errno));
			}
			if(buffer[0] == 'q')
				listen =false;	
			printf("\ncontents: %s", buffer);
		}	

		xbee.close_port();
	}
*/

}
