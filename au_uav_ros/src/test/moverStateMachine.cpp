//AT LEAST a 1.2 second delay when it stops sending commands
//let's test mover. make sure that thing WORKS


#include <vector>
#include <iostream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <au_uav_ros/pi_standard_defs.h>
#include <roscpp/Logger.h>
#include <au_uav_ros/Command.h>

#include <boost/thread.hpp>

//Really can't think of how to handle the plane ID delimma. Just don't get the planeID.. comment that bit out
class test_state_machine {
	public:
	test_state_machine()	{
		sinceLastCACommand = ros::Time::now();
		lag.sec = 1;
		sleep_time = 1.3;
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
		ros::Time arrival = ros::Time::now();
		fprintf(stderr, "tester::GOT CA COMMAND CALLBACK!");	
		fprintf(stderr, "\traw:: lat: %f|lon: %f|alt: %f\n",com.latitude, com.longitude, com.altitude);
		sinceLastCACommand = ros::Time::now();	

		
		//Only push back if it's CHANGED (alg continually outputs goal_wp)  and not default (0.0)
		if(com.latitude != 0 || previousCom.latitude != com.latitude)	{ 
			fprintf(stderr, "tester::callback -> GASP it's a new ca_command!\n");
			fprintf(stderr, "\tnew:: lat: %f|lon: %f|alt: %f\n",com.latitude, com.longitude, com.altitude);
			queue_lock.lock();
			ca_commands_bin.push_back(com);
			time_bin.push_back(arrival);	
			queue_lock.unlock();
		}
		previousCom = com;
	}

	bool ca_commands_chatty()	{
		return ros::Time::now() - sinceLastCACommand < lag; 
	}

	bool ca_commands_silent()	{
		return ros::Time::now() - sinceLastCACommand > lag; 
	}

	std::vector<au_uav_ros::Command> getReceivedCommands()	{
		queue_lock.lock();
		std::vector<au_uav_ros::Command> temp(ca_commands_bin);
		queue_lock.unlock();
		return temp;
	}
	std::vector<ros::Time> getReceivedTimes()	{
		queue_lock.lock();
		std::vector<ros::Time> temp(time_bin);
		queue_lock.unlock();
		return temp;
	}
	boost::mutex queue_lock;

	//stuff
	ros::Time sinceLastCACommand;	
	ros::Duration lag;	//one second lag OK from sending command to reponse

	au_uav_ros::Command stop;
	au_uav_ros::Command go;
	au_uav_ros::Command regCom;

	au_uav_ros::Command previousCom;			//don't want to pushback duplicate commands

	std::vector<au_uav_ros::Command> ca_commands_bin;	// all the ca waypoints i've gotten.
	std::vector<ros::Time> time_bin;			// time that ca_waypoints came in. 
	double sleep_time; 

};

void spinThread()	{
	fprintf(stderr, "spinning...");
	ros::spin();
}



TEST(mover, state_runthrough)	{


	test_state_machine t;
	ros::NodeHandle n;
	ros::Publisher gcs = n.advertise<au_uav_ros::Command>("gcs_commands", 5);	
	
	ros::Subscriber ca = n.subscribe("ca_commands", 1, &test_state_machine::ca_command_callback, &t);	
	
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
	
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	
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
	
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent());	

}


//Given that avoidance is turned off, feed goal wps to mover.
//om nom nom

TEST(mover, no_avoid_feeding)	{

	ros::Duration(7).sleep(); //let spinner thread startup 
	
	
	test_state_machine t;
	ros::NodeHandle n;	
	int numWaypoints = 5;
	ros::Publisher gcs = n.advertise<au_uav_ros::Command>("gcs_commands", 5);	
	ros::Subscriber ca = n.subscribe("ca_commands", 10, &test_state_machine::ca_command_callback, &t);	
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent());	
	gcs.publish(t.go); //transition
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_chatty());	



	
	au_uav_ros::Command com;
	com.latitude = 10; com.longitude = 10; com.altitude = 10; com.planeID = 999;	
	std::vector<au_uav_ros::Command> forceFeeding;
	for(int i =0; i< numWaypoints; i++)	{
		forceFeeding.push_back(com);
		com.latitude++; com.longitude++; com.altitude++;
		fprintf(stderr, "test::Pushed back lat: %f| long: %f| alt: %f\n", forceFeeding[i].latitude,
										forceFeeding[i].longitude,
										forceFeeding[i].altitude);
	}

	std::vector<ros::Time> sendingTimes;
	for(int i =0; i< numWaypoints; i++)	{
		fprintf(stderr, "test::GCS sent lat: %f| long: %f| alt: %f\n", forceFeeding[i].latitude,
										forceFeeding[i].longitude,
										forceFeeding[i].altitude);
		gcs.publish(forceFeeding[i]);
		sendingTimes.push_back(ros::Time::now());	
		ros::Duration(.25).sleep();	//time enough to measure lag initially
	}

	fprintf(stderr, "STOP STOP!\n");	
	gcs.publish(t.stop); //publish stop... transition back
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent());	

	
//	fprintf(stderr, "sleeping");
	//ros::Duration(10).sleep(); //let mover get moving

	//Now, seeing if the bins are equal...

	fprintf(stderr, "Trying to get received...\n");	
	std::vector<au_uav_ros::Command> ca_commands_bin = t.getReceivedCommands();	
	std::vector<ros::Time> time_bin = t.getReceivedTimes();		

	
	for(int i =0; i< numWaypoints; i++)	{
		fprintf(stderr, "received :: back lat: %f \n", ca_commands_bin[i].latitude);
//		EXPECT_TRUE(forceFeeding[i].latitude == ca_commands_bin[i].latitude) << "differ: " << forceFeeding[i] << "and " << ca_commands_bin[i]; 
//		EXPECT_TRUE(forceFeeding[i].longitude == ca_commands_bin[i].longitude) << "differ: " << forceFeeding[i] << "and " << ca_commands_bin[i]; 
//		EXPECT_TRUE(forceFeeding[i].altitude == ca_commands_bin[i].altitude) << "differ: " << forceFeeding[i] << "and " << ca_commands_bin[i]; 
//		fprintf(stderr, " Time diff (s) command %d: %f\n\n", i, (time_bin[i] - sendingTimes[i]).toSec());	
	}		


	fprintf(stderr, "test::TRYING TO SHTUDOWN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

}


int main(int argc, char ** argv)	{
	ros::init(argc, argv, "planeIDTester");
	testing::InitGoogleTest(&argc, argv);
	boost::thread spinner(spinThread);
	ros::Time::init();
	RUN_ALL_TESTS();
	ros::shutdown();
	spinner.join();
}
