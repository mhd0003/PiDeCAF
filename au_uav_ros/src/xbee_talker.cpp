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
		return false;
	}
	else	{
		ROS_INFO("opened port %s", m_port.c_str());
		m_xbee.setup_port(m_baud, 8, 1, true);		
	}

	//mavlink
	sysid = -1;
	compid = 110;	//not entirely sure why this is 110
	serial_compid =0;	//the compid that the GCS is getting from plane (so we won't need)
	pc2serial = true;
	updateIndex = 0;
	WPSendSeqNum = 0;

	//Set up Ros stuff. Todo - update this my_telemetry
	m_node = _n;
	telem_sub = m_node.subscribe("telemetry", 10, &XbeeTalker::myTelemCallback, this); 
	return true;
}

void au_uav_ros::XbeeTalker::run()	{
	ROS_INFO("Entering Run");

	//Spin up thread to execute myTelemCallback()
	boost::thread broadcastMyTelem(boost::bind(&XbeeTalker::spinThread, this));	

	//Start listening for telemetry and commands, upon receiving proper command
	//from ground station, execute shutdown and join
	listen();

	ros::shutdown();
	broadcastMyTelem.join();	
}

void au_uav_ros::XbeeTalker::shutdown()	{
	//ROS_INFO("Shutting down Xbee port %s", m_port.c_str());
	printf("XbeeTalker::Shutting down Xbee port %s\n", m_port.c_str()); //i ros::shutdown when exiting run
	m_xbee.close_port();
}

bool au_uav_ros::XbeeTalker::convertROSToMavlinkTelemetry(au_uav_ros::Telemetry &tUpdate, mavlink_au_uav_t &mavMessage)	{
	
}

bool au_uav_ros::XbeeTalker::convertMavlinkTelemetryToROS(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate) {
	//tUpdate.planeID = mavlink_ros.planeID;
	
	tUpdate.currentLatitude = (mavMessage.au_lat) / 10000000.0;	  /// Lattitude * 10**7 so have to divide by 10^7
	tUpdate.currentLongitude = (mavMessage.au_lng) / 10000000.0;	  /// Longitude * 10**7 so have to divide by 10^7
	tUpdate.currentAltitude = (mavMessage.au_alt) / 100.0; 	  /// Altitude in cm so divide by 100 to get meters	
	
	tUpdate.destLatitude = (mavMessage.au_target_lat) / 10000000.0;  /// Lattitude * 10**7 so have to divide by 10^7
	tUpdate.destLongitude = (mavMessage.au_target_lng) / 10000000.0; /// Longitude * 10**7 so have to divide by 10^7
	tUpdate.destAltitude = (mavMessage.au_target_alt) / 100.0; 	  /// Altitude in cm so divide by 100 to get meters

	tUpdate.groundSpeed = (mavMessage.au_ground_speed) / 100.0; /// Originally in cm / sec so have to convert to mph

	tUpdate.distanceToDestination = mavMessage.au_distance; 	  /// Distance between plane and next waypoint in meters.

	tUpdate.targetBearing = mavMessage.au_target_bearing / 100;	  /// This is the direction to the next waypoint or loiter center in degrees

	tUpdate.currentWaypointIndex = mavMessage.au_target_wp_index;   /// The current waypoint index

	tUpdate.telemetryHeader.seq = ++updateIndex;
	tUpdate.telemetryHeader.stamp = ros::Time::now();
	return true;
}

//Input - listening to xbee for other telem msgs and gcs commands
//---------------------------------------------------------------------------

//acccck - letting andrew look at this

void au_uav_ros::XbeeTalker::listen()	{

//Used for testing	
	char buffer[256];
	memset(buffer, '\0', 256);
	
	
	bool keepListening= true;

	while(keepListening && ros::ok())	{
		if(read(m_xbee.getFD(), buffer, 256) == -1)	{
			ROS_INFO("error in read fd %s\n", strerror(errno));	
		}	
		if(buffer[0] == 'q')
			keepListening= false;
		printf("%s", buffer);
		memset(buffer, '\0', 256);
	}




}

//Output - writing to xbee
//---------------------------------------------------------------------------
void au_uav_ros::XbeeTalker::myTelemCallback(au_uav_ros::Telemetry tUpdate)	{
	ROS_INFO("XbeeTalker::telemCallback::ding! \n");

	
}

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
	if(talk.init(n))	//OK. what if the xbee can't be read???
		talk.run();
	talk.shutdown();
}
