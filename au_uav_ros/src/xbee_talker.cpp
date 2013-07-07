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
	m_telem_pub = m_node.advertise<au_uav_ros::Telemetry>("all_telemetry", 5);
	telem_sub = m_node.subscribe("my_mav_telemetry", 10, &XbeeTalker::myTelemCallback, this); 
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

//Input - listening to xbee for other telem msgs and gcs commands
//---------------------------------------------------------------------------

//acccck - letting andrew look at this

void au_uav_ros::XbeeTalker::listen()	{

//Used for testing
	/*
	char buffer[256];
	memset(buffer, '\0', 256);
	bool keepListening= true;

	while(keepListening)	{
		if(read(m_xbee.getFD(), buffer, 256) == -1)	{
			ROS_INFO("error in read fd %s\n", strerror(errno));	
		}	
		if(buffer[0] == 'q')
			keepListening= false;
		printf("%s", buffer);
		memset(buffer, '\0', 256);
	}
	*/
        while(ros::ok())
        {
                //get a mavlink message from the serial line
                mavlink_message_t message = au_uav_ros::mav::readMavlinkFromSerial(m_xbee);
                //decode the message and post it to the appropriate topic

		 if(message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
                {
                        mavlink_heartbeat_t receivedHeartbeat;
                        mavlink_msg_heartbeat_decode(&message, &receivedHeartbeat);
                        ROS_INFO("Received heartbeat");
                }
                if(message.msgid == MAVLINK_MSG_ID_AU_UAV)
                {
                        //ROS_INFO("Received AU_UAV message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, mes$
                        au_uav_ros::Telemetry tUpdate;
                        mavlink_au_uav_t myMSG;
                        mavlink_msg_au_uav_decode(&message, &myMSG);            // decode generic mavlink message in$
                        au_uav_ros::mav::convertMavlinkTelemetryToROS(myMSG, tUpdate);                   // decode AU_UAV ma$
                        tUpdate.planeID = message.sysid;                                // update planeID
                        m_telem_pub.publish(tUpdate);
			ROS_INFO("Received telemetry message from UAV[#%d] (lat:%f|lng:%f|alt:%f)", tUpdate.planeID,
					 tUpdate.currentLatitude, tUpdate.currentLongitude, tUpdate.currentAltitude);
                }
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
	int baud = 57600;
	std::cout << "helo world!" <<std::endl;
	std::string port = "/dev/ttyUSB0";
	if(argc == 2){
		baud = atoi(argv[1]);
	}
	ros::init(argc, argv, "XbeeTalker");
	ros::NodeHandle n;
	au_uav_ros::XbeeTalker talk(port, baud);
	if(talk.init(n))	//OK. what if the xbee can't be read???
		talk.run();
	talk.shutdown();
}
