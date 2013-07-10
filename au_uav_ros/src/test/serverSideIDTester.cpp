#include <iostream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <au_uav_ros/planeIDGetter.h>

namespace {
class test_id_srv : public :: testing::Test	{
	protected:

	virtual void SetUp()	{
		client = n.serviceClient<au_uav_ros::planeIDGetter>("getPlaneID");		
			
	}

	virtual void TearDown()	{
		ros::shutdown();
	}

	//stuff
	ros::NodeHandle n;
	ros::ServiceClient client;

};


}

TEST_F(test_id_srv, getPlaneID)	{

	//wow this is confusing.
	//So, if the ardupilot port can't even be opened, the id will be returned as 0??? what's happening?

	//ros::Duration(30).sleep(); //The other callback is saying it's got 32, but I'm getting a FALSE for client call success.

//	ASSERT_TRUE(ros::ok());	//if xbee can't be read, ardu should call shutdown
	au_uav_ros::planeIDGetter srv;
	EXPECT_TRUE(client.call(srv)) << "Client Call DID NOT Succeed";
	EXPECT_FALSE(srv.response.planeID == -1);
	EXPECT_TRUE(srv.response.planeID == (long int)32);

	fprintf(stderr, "planeIDTESTER::Plane ID is %d", srv.response.planeID);

}


int main(int argc, char ** argv)	{
	ros::init(argc, argv, "planeIDTester");
	testing::InitGoogleTest(&argc, argv);
	ros::Time::init();
	RUN_ALL_TESTS();
}
