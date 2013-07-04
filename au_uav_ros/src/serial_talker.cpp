#include "au_uav_ros/serial_talker.h"

/*

void XbeeIn::setup() {
	shutdownTopic = n.subscribe("component_shutdown", 1000, &XbeeIn::component_shutdown, this);

	initCommsService = n.advertiseService("init_incomms", &XbeeIn::init_comms, this);
	closeCommsService = n.advertiseService("close_incomms", &XbeeIn::close_comms, this);

	telemetryTopic = n.advertise<au_uav_ros::Telemetry>("telemetry", 1000);

	sendAckClient = n.serviceClient<au_uav_ros::Ack>("ack_recieved");
	recieveAckService = n.advertiseService("add_ack", &XbeeIn::add_ack, this);

	m_fd = -1;
	baud = 57600;
	sysid = -1;
	compid = 110;
	serial_compid = 0;
	m_port = "/dev/ttyUSB0";
	pc2serial = true;
	updateIndex = 0;
	WPSendSeqNum = 0;
}
*/

SerialTalker::SerialTalker()	{
	m_fd = -1;
	m_port= ""; 
}


int SerialTalker::open_port(std::string _port)
{
	int fd; /* File descriptor for the port */
	
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	// O_NDELAY - use Non blocking IO.
	m_port = _port;
	fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		/* Could not open the port. */
//		ROS_ERROR("IN: COULD NOT OPEN PORT");
		fprintf(stderr, "\nport %s  could not be opened\n", _port.c_str());
		return(-1);
	}
	else
	{
		//I want this to block!!
		fcntl(fd, F_SETFL, 0);
	}

	m_fd = fd;
	fprintf(stderr, "\nport %s opened\n", _port.c_str());
	return (m_fd);
}

bool SerialTalker::setup_port(int baud, int data_bits, int stop_bits, bool parity )
{
	//struct termios options;
	
	struct termios  config;
	if(!isatty(m_fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %s is NOT a serial port\n", m_port.c_str());
		return false;
	}
	if(tcgetattr(m_fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of port %s\n", m_port.c_str());
		return false;
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	config.c_oflag = 0;
	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	
	//Note: Returns as soon as input is available, or if VTIME expires, returns with no chars. -> 
	//Apparently, conflicts with modem hangup and EOF?? TODO
	config.c_cc[VMIN]  = 0; //TODO changed from 1 to 0 it was blocking read so it could never get shutdown if no message was sent
	config.c_cc[VTIME] = 10; // was 0
	
	// Get the current options for the port
	//tcgetattr(m_fd, &options);
	
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, falling back to 115200 8N1 default rate.\n", baud);
			cfsetispeed(&config, B115200);
			cfsetospeed(&config, B115200);
			
			break;
	}
	
	//
	// Finally, apply the configuration
	//
	if(tcsetattr(m_fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nSerialTalker::ERROR: could not set configuration of port %s\n", m_port.c_str());
		return false;
	}
	fprintf(stderr, "\nSerialtalker::configuration of port %s set to baud %d\n", m_port.c_str(), baud);
	return true;
}

bool SerialTalker::close_port()
{
	close(m_fd);
	fprintf(stderr, "\nSerialtalker::port %s closed.\n", m_port.c_str());
	return true;
}
/*
bool XbeeIn::convertMavlinkTelemetryToROS(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate) {
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
*/
