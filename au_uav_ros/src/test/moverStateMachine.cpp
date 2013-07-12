//AT LEAST a 1.2 second delay when it stops sending commands


#include <iostream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <au_uav_ros/pi_standard_defs.h>
#include <roscpp/Logger.h>
#include <au_uav_ros/Command.h>

#include <boost/thread.hpp>

//Really can't think of how to handle the plane ID delimma. Just don't get the planeID.. comment that bit out
class test_id_srv {
	public:
	test_id_srv()	{
		sinceLastCACommand = ros::Time::now();
		lag.sec = 1;

		stop.latitude = EMERGENCY_PROTOCOL_LAT;
		stop.longitude = EMERGENCY_STOP_LON;
		stop.planeID = 999;

		go.latitude = EMERGENCY_PROTOCOL_LAT;
		go.longitude = EMERGENCY_START_LON;
		go.planeID = 999;

		regCom.latitude = 123;
		regCom.longitude = 123;
		regCom.planeID = 999;
	}


	void ca_command_callback(au_uav_ros::Command com)	{
		fprintf(stderr, "tester::GOT CA COMMAND CALLBACK!\n");	
		sinceLastCACommand = ros::Time::now();	
	}

	bool ca_commands_chatty()	{
		return ros::Time::now() - sinceLastCACommand < lag; 
	}

	bool ca_commands_silent()	{
		return ros::Time::now() - sinceLastCACommand > lag; 
	}
	//stuff
	ros::Time sinceLastCACommand;	
	ros::Duration lag;	//one second lag OK from sending command to reponse

	au_uav_ros::Command stop;
	au_uav_ros::Command go;
	au_uav_ros::Command regCom;
};

void spinThread()	{
	fprintf(stderr, "spinning...");
	ros::spin();
}



TEST(mover, state_runthrough)	{

	double sleep_time = 1.3;
	boost::thread spinner(spinThread);

	test_id_srv t;
	ros::NodeHandle n;
	ros::Publisher gcs = n.advertise<au_uav_ros::Command>("gcs_commands", 5);	
	
	ros::Subscriber ca = n.subscribe("ca_commands", 1, &test_id_srv::ca_command_callback, &t);	
	
	ros::Publisher fake = n.advertise<au_uav_ros::Command>("ca_commands", 5);	
	fake.publish(t.regCom);
	fprintf(stderr, "test::FAKE PUBLISH");
	ros::Duration(5).sleep();	//time enough to measure lag initially

	//NO GO MODE
	//-------------
	//Mover should be starting in NOGO mode.
	fprintf(stderr, "TEST::Expect start in NOGO\n");
	EXPECT_TRUE(t.ca_commands_silent());	//No commands should have been sent within the last lag period.


	gcs.publish(t.stop); //publish stop...
	EXPECT_TRUE(t.ca_commands_silent());	

	gcs.publish(t.regCom);	//publish regular command
	EXPECT_TRUE(t.ca_commands_silent());	

	gcs.publish(t.go); //transition
	
	ros::Duration(sleep_time).sleep();	//time enough to measure lag initially
	
	EXPECT_TRUE(t.ca_commands_chatty());	

	fprintf(stderr, "TEST::Expect next stage is OK\n");
	//OK MODE
	//-----------	
	//IN OK mode... testing all inputs
	gcs.publish(t.go);
	EXPECT_TRUE(t.ca_commands_chatty()); 

	gcs.publish(t.regCom);
	EXPECT_TRUE(t.ca_commands_chatty());	

	gcs.publish(t.stop); //publish stop... transition back
	
	ros::Duration(sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent());	

	ros::shutdown();
	spinner.join();

}



int main(int argc, char ** argv)	{
	ros::init(argc, argv, "planeIDTester");
	testing::InitGoogleTest(&argc, argv);
	ros::Time::init();
	RUN_ALL_TESTS();
}
