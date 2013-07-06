#ifndef SERIAL_TALKER_H
#define SERIAL_TALKER_H 

#include <unistd.h> /*fcntl - manip file descriptor */
#include <string>

// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <boost/thread.hpp>	//locks

#ifdef __linux
#include <sys/ioctl.h>
#endif

//#include "mavlink/v1.0/common/mavlink.h"

class SerialTalker{
private:
/*
	int sysid;
	int compid;
	int serial_compid;
	std::string port;
	bool pc2serial;
*/
	std::string m_port;
	int m_fd;
	int m_baud;

	boost::mutex m_serialLock;

public:
	SerialTalker();
	
	/*
	 * Open port
	 * Attempts to open given port. Returns -1 if unsuccesfull. 
	 */
	int open_port(std::string port);
	/*
	 * Setup Port
	 * Changes baud rate.
	 * Magically set up some other stuff too. TODO: actually change data_bits, etc. 
	 */
	bool setup_port(int baud, int data_bits, int stop_bits, bool parity);
	/*
	 * Close Port
	 */
	bool close_port();

	//Getters
	int getFD()	{return m_fd;}	
	std::string getPortName() {return m_port;}
	int getBaud()	{return m_baud;}
	//Locking
	void lock()	{ m_serialLock.lock();}	
	void unlock()	{ m_serialLock.unlock();}	
	//bool convertMavlinkTelemetryToROS(mavlink_au_uav_t &mavMessage, au_uav_ros::Telemetry &tUpdate);
	//void* serial_wait(void* serial_ptr);
};

#endif
