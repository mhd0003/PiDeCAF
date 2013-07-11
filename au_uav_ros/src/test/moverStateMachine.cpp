#include <iostream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <au_uav_ros/planeIDGetter.h>

//Really can't think of how to handle the plane ID delimma. Just don't get the planeID.. comment that bit out

namespace {
class test_id_srv : public :: testing::Test	{
	protected:
	
	bool get(au_uav_ros::planeIDGetter::Request &req,
			au_uav_ros::planeIDGetter::Response &res)	{
			res.planeID = 9999;	
	}

	virtual void SetUp()	{
		server = n.advertiseService("getPlaneID", &test_id_srv::get, this);		
			
	}

	virtual void TearDown()	{
		ros::shutdown();
	}

	//stuff
	ros::NodeHandle n;
	ros::ServiceServer server;

};


}

/*
TEST_F(test_id_srv, returnPlaneID)	{

	//wow this is confusing.
	//So, if the ardupilot port can't even be opened, the id will be returned as 0??? what's happening?
	ros::Duration(30).sleep(); 
//	EXPECT_TRUE(
	fprintf(stderr, "planeIDTESTER::Plane ID is %d", srv.response.planeID);

}
*/


int main(int argc, char ** argv)	{
	ros::init(argc, argv, "planeIDTester");
	testing::InitGoogleTest(&argc, argv);
	ros::Time::init();
	RUN_ALL_TESTS();
}
