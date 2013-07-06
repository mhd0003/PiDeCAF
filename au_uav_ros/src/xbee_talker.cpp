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

	while(keepListening)	{
		if(read(m_xbee.getFD(), buffer, 256) == -1)	{
			ROS_INFO("error in read fd %s\n", strerror(errno));	
		}	
		if(buffer[0] == 'q')
			keepListening= false;
		printf("%s", buffer);
		memset(buffer, '\0', 256);
	}



	/*
	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;

	
	// Blocking wait for new data
	while (ros::ok())
	{
		//if (debug) printf("Checking for new data on serial port\n");
		// Block until data is available, read only one byte to be able to continue immediately
		//char buf[MAVLINK_MAX_PACKET_LEN];
		uint8_t cp;
		mavlink_message_t message;
		mavlink_status_t status;
		uint8_t msgReceived = false;
		//tcflush(fd, TCIFLUSH);
		xbeeLock.lock();
		int num = read(m_xbee.getFD(), &cp, 1);
		xbeeLock.unlock();
		
		if (num > 0)
		{
			// Check if a message could be decoded, return the message in case yes
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
			if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)
			{
				if (verbose || debug) printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
				if (debug)
				{
					unsigned char v=cp;
					fprintf(stderr,"%02x ", v);
				} 
			}
			lastStatus = status;
		}
		else
		{
			//if (!silent) fprintf(stderr, "ERROR: Could not read from port %s\n", port.c_str());
		}
		
		// If a message could be decoded, handle it
		if(msgReceived)
		{
			//if (verbose || debug) std::cout << std::dec << "Received and forwarded serial port message with id " << static_cast<unsigned int>(message.msgid) << " from system " << static_cast<int>(message.sysid) << std::endl;
			//if (debug) ROS_ERROR("Message sequence #: %d", message.seq);
			// DEBUG output
			if (debug)
			{
				fprintf(stderr,"Forwarding SERIAL -> ROS: ");
				unsigned int i;
				uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
				unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
				if (messageLength > MAVLINK_MAX_PACKET_LEN)
				{
					fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
				}
				else
				{
					for (i=0; i<messageLength; i++)
					{
						unsigned char v=buffer[i];
						fprintf(stderr,"%02x ", v);
					}
					fprintf(stderr,"\n");
				}
			}
			
			if (verbose || debug)
				ROS_INFO("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
			
			
			//TODO: Need better way to keep track of multiple UAVs, this should work fine for one UAV.
			//	Maybe keep an array of current 'real' UAVs			
			sysid = message.sysid;
			serial_compid = message.compid;

			
			// Serialize the Mavlink-ROS-message
			 

			//mavlink_ros::Mavlink rosmavlink_msg;

			//rosmavlink_msg.len = message.len;
			//rosmavlink_msg.seq = message.seq;
			//rosmavlink_msg.sysid = message.sysid;
			//rosmavlink_msg.compid = message.compid;
			//rosmavlink_msg.msgid = message.msgid;
			//rosmavlink_msg.fromlcm = false;

			//if (verbose) 
			//	myMessages[message.msgid]++;

//			for (int i = 0; i < message.len/8; i++)
//			{
//				(rosmavlink_msg.payload64).push_back(message.payload64[i]);
//			}
			
			 // Mark the ROS-Message as coming not from LCM
			
			//rosmavlink_msg.fromlcm = true;
			
			 //Send the received MAVLink message to ROS (topic: mavlink, see main())
			 
			//mavlink_pub.publish(rosmavlink_msg);

			//ROS_ERROR("ID %d", message.msgid);

			if(message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
			{
				//ROS_INFO("Delay: %d", delay);
				//mavlink_heartbeat_t receivedHeartbeat;
				//mavlink_msg_heartbeat_decode(&message, &receivedHeartbeat);
				//ROS_INFO("Received heartbeat with ID #%d (type:%d | AP:%d | base:%d | custom:%d | status:%d)\n", message.msgid, receivedHeartbeat.type, receivedHeartbeat.autopilot, receivedHeartbeat.base_mode, receivedHeartbeat.custom_mode, receivedHeartbeat.system_status);
			}
			if(message.msgid == MAVLINK_MSG_ID_AU_UAV)
			{
				//ROS_INFO("Received AU_UAV message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
				
				au_uav_ros::Telemetry tUpdate;
				mavlink_au_uav_t myMSG;
				mavlink_msg_au_uav_decode(&message, &myMSG);		// decode generic mavlink message into AU_UAV message
				convertMavlinkTelemetryToROS(myMSG, tUpdate);			// decode AU_UAV mavlink struct into ROS message
				tUpdate.planeID = message.sysid;				// update planeID
				ROS_INFO("Received telemetry message from UAV[#%d] (lat:%f|lng:%f|alt:%f)", tUpdate.planeID, tUpdate.currentLatitude, tUpdate.currentLongitude, tUpdate.currentAltitude);
				//telemetryTopic.publish(tUpdate);	// publish ROS telemetry message to ROS topic /telemetry
			}
			//TODO: add code here to test if received ack corresponsed to desired target_wp, if not retransmit target_wp
		
		//Not entirely sure what this is acking for. Not needed here hopefully. 
				
	//		if (message.msgid == MAVLINK_MSG_ID_MISSION_ACK) 
//			{
//				ROS_ERROR("NRO");
//				mavlink_mission_ack_t mission_ack;
//				mavlink_msg_mission_ack_decode(&message, &mission_ack);
//				ROS_INFO("RECEIVED message from serial with ID #%d (sys:%d|comp:%d|type:%d):\n", 
//						message.msgid, message.sysid, message.compid, mission_ack.type);
//				Ack a;
//				a.request.planeID = message.sysid;
//				sendAckClient.call(a);
//			}
			
		}
	}
	

*/




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
