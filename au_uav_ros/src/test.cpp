#include "au_uav_ros/serial_talker.h"
#include <cstdio> //printf
#include <string>
#include <ros/console.h>	//used for debugging
int main()	{
	
	std::string port = "/dev/ttyUSB0";
	SerialTalker xbee;

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


}
