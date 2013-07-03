#ifndef SERIAL_TALKER_H
#define SERIAL_TALKRE_H 

//#include <iostream>
//#include <cstdlib>
#include <unistd.h> /*fcntl - manip file descriptor */
//#include <cmath>
#include <string>
//#include <inttypes.h>
//#include <fstream>

// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#ifdef __linux
#include <sys/ioctl.h>
#endif

//#include <glib.h>

// Latency Benchmarking
//#include <sys/time.h>
//#include <time.h>

//Standard C++ headers
//#include <sstream>

//#include "mavlink/v1.0/common/mavlink.h"

namespace au_uav_ros{
	class Serial_talker{
	private:
/*
		int baud;
		int sysid;
		int compid;
		int serial_compid;
		std::string port;
		bool pc2serial;
*/
		std::string port;
		int fd;
/*		int updateIndex;
		int WPSendSeqNum;
		int myMessage[256];
*/
	public:
		bool setup_port(int baud, int data_bits, int stop_bits, bool parity);
		int open_port(std::string port);
		bool close_port();
		//bool convertMavlinkTelemetryToROS(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate);
		void* serial_wait(void* serial_ptr);
	};
}
#endif
