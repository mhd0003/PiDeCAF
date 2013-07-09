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



	//stuff
	ros::NodeHandle n;
	ros::ServiceClient client;
	


};





}

TEST_F(test_id_srv, getPlaneID)	{

	//wow this is confusing.
	//So, if the ardupilot port can't even be opened, the id will be returned as 0??? what's happening?
	au_uav_ros::planeIDGetter srv;
	client.call(srv);
	EXPECT_FALSE(srv.response.planeID == -1) << "PlaneID is" << srv.response.planeID;

	std::cout << "Plane id is " << srv.response.planeID;	
}


int main(int argc, char ** argv)	{
	ros::init(argc, argv, "planeIDTester");
	testing::InitGoogleTest(&argc, argv);
	ros::Time::init();
	RUN_ALL_TESTS();
}
