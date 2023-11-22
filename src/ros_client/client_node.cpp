#include <knowrob/ros_client/cpp/knowrob_client.hpp>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "KnowrobClientNode");
   KnowrobClient kc;

   ros::spin();
   exit(0);
}