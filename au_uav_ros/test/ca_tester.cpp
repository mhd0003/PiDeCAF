#include <gtest/gtest.h>

//Unit testing!

namespace	{

class GetPlaneIDTester: public ::testing::Test	{
	protected:
		GetPlaneIDTester()	{
		
		}

		//more stuff

};//end GetPlaneIDTester class

TEST_F(GetPlaneIDTester, something)	{

}


}





int main (int argc, char ** argv)	{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
