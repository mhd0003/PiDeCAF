#ifndef XBEE_TALKER_H
#define XBEE_TALKER_H

#include "au_uav_ros/serial_talker.h"

namespace au_uav_ros{
	class XbeeTalker	{
	private:
		SerialTalker m_xbee;			
		std::string m_port;
		int m_baud;
	public:
		XbeeTalker();
		XbeeTalker(std::string port, int baud);
		/*
		 * init
		 * Opens port.
		 */
		bool init();

		void shutdown();
	};
}
#endif
