#include <knowrob/ros_client/cpp/knowrob_client.hpp>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "KnowrobClientNode");
   std::string logger_prefix = std::string("[KCN]\t");

   ROS_INFO_STREAM(logger_prefix << "Starting the knowrob client node..");

   ros::NodeHandle nh;
   KnowrobClient kc;
   kc.initialize(nh);

   

   // first ask
   KnowrobQuery single_query = kc.getDefaultQueryMessage();
   KnowrobAnswer single_answer;
   single_query.queryString = std::string("test:hasSon(A,B)");
   
   bool result = kc.askOne(single_query, single_answer);

   if(!result)
   {
      ROS_ERROR_STREAM(logger_prefix << "AskOne false!");
   }
   else
   {
      ROS_INFO_STREAM(logger_prefix << "AskOne true!");
      ROS_INFO_STREAM(logger_prefix << "Answer is: " << single_answer.instantiation);
   }


   // first tell
   KnowrobQuery single_tell = kc.getDefaultQueryMessage();
   single_tell.queryString = std::string("test:hasSon(a,b)");
   
   result = kc.tell(single_tell);

   if(!result)
   {
      ROS_ERROR_STREAM(logger_prefix << "Tell false!");
   }
   else
   {
      ROS_INFO_STREAM(logger_prefix << "Tell true!");
   }

   
   exit(0);
}