#include "au_uav_ros/ardu_talker.h"
#include <cstdio> //printf
#include <string>
#include <ros/console.h>	//used for debugging

au_uav_ros::ArduTalker::ArduTalker()	{
	m_port = "dev/ttyUSB0";
	m_baud = 9600;
}

au_uav_ros::ArduTalker::ArduTalker(std::string _port, int _baud)	{
	m_port = _port;
	m_baud = _baud;	
	
}

bool au_uav_ros::ArduTalker::init(ros::NodeHandle _n)	{
	//Open and setup port.
	if(m_ardu.open_port(m_port) == -1)	{
		ROS_INFO("Could not open port %s", m_port.c_str());
		return false;
	}
	else	{
		ROS_INFO("opened port %s", m_port.c_str());
		m_ardu.setup_port(m_baud, 8, 1, true);		
	}

	//mavlink
	sysid = -1;
	compid = 110;	//not entirely sure why this is 110
	serial_compid =0;	//the compid that the GCS is getting from plane (so we won't need)
	pc2serial = true;
	updateIndex = 0;
	WPSendSeqNum = 0;

	//Set up Ros stuff. Todo - 
	m_node = _n;
	m_command_sub = m_node.subscribe("ca_command", 10, &ArduTalker::commandCallback, this); 
	
	m_telem_pub = m_node.advertise<au_uav_ros::Telemetry>("all_telemetry", 5);
	m_mav_telem_pub = m_node.advertise<au_uav_ros::Telemetry>("my_mav_telemetry", 5);//the type will need to change
	return true;
}

void au_uav_ros::ArduTalker::run()	{
	ROS_INFO("Entering Run");

	//Spin up thread to execute myTelemCallback()
	boost::thread sendCommands(boost::bind(&ArduTalker::spinThread, this));	

	//Start listening for telemetry and commands, upon receiving proper command
	//from ground station, execute shutdown and join
	listen();

	ros::shutdown();
	sendCommands.join();	
}

void au_uav_ros::ArduTalker::shutdown()	{
	//ROS_INFO("Shutting down Xbee port %s", m_port.c_str());
	printf("ArduTalker::Shutting down Xbee port %s\n", m_port.c_str()); //i ros::shutdown when exiting run
	m_ardu.close_port();
}

//Input - listening ardu for other telem msgs and gcs commands
//---------------------------------------------------------------------------

void au_uav_ros::ArduTalker::listen()	{

//Used for testing	
	char buffer[256];
	memset(buffer, '\0', 256);
	
	
	bool keepListening= true;

	while(keepListening)	{
		if(read(m_ardu.getFD(), buffer, 256) == -1)	{
			ROS_INFO("error in read fd %s\n", strerror(errno));	
		}	
		if(buffer[0] == 'q')
			keepListening= false;
		printf("%s", buffer);
		memset(buffer, '\0', 256);
	}


	//publish to m_telem_pub and m_mav_telem_pub 


}

//Output - writing to xbee
//---------------------------------------------------------------------------
void au_uav_ros::ArduTalker::commandCallback(au_uav_ros::Command update)	{
	ROS_INFO("ArduTalker::commandCallback::ding! \n");
	//-----------------
	//Callback time! 
	//----------------

	//take command -> send to ardupilot	
}

void au_uav_ros::ArduTalker::spinThread()	{
	ROS_INFO("ArduTalker::spinThread::Starting spinner thread");
	//Handle myTelemCallback()
	ros::spin();	
}

int main(int argc, char** argv)	{

	std::cout << "helo world!" <<std::endl;
	std::string port = "/dev/ttyUSB0";

	ros::init(argc, argv, "ArduTalker");
	ros::NodeHandle n;
	au_uav_ros::ArduTalker talk(port, 9600);
	if(talk.init(n))	//OK. what if the xbee can't be read???
		talk.run();
	talk.shutdown();
}
