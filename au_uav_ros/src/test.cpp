#include "au_uav_ros/Command.h"

#include "ros/ros.h"

using namespace au_uav_ros;

int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<au_uav_ros::Command>("commands", 1000);

	ros::Rate r(10);
	Command cmd;
	cmd.planeID = 36;
	cmd.altitude = 0;
	cmd.longitude = 0;
	cmd.latitude = 0;
	cmd.commandID = 2;
	cmd.param = 1;

	while (ros::ok()) {
		pub.publish(cmd);
		r.sleep();
	}
}
