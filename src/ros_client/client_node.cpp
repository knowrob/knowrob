#include <knowrob/ros_client/cpp/knowrob_client.hpp>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "KnowrobClientNode");
   std::string logger_prefix = std::string("[KCN]\t");

   ROS_INFO_STREAM(logger_prefix << "Starting the knowrob client node..");
   ros::Duration(1.0).sleep();


   ros::NodeHandle nh;
   KnowrobClient kc;
   kc.initialize(nh);  

   // first ask
   KnowrobQuery single_query = kc.getDefaultQueryMessage();
   KnowrobAnswer single_answer;
   std::vector<KnowrobAnswer> multi_answer;
   single_query.queryString = std::string("test:hasSon(A,B)");
   
   ROS_INFO_STREAM(logger_prefix << "Sending AskAll Query!");
   bool result = kc.askAll(single_query, multi_answer);

   if(!result)
   {
      ROS_ERROR_STREAM(logger_prefix << "AskAll false!");
   }
   else
   {
      ROS_INFO_STREAM(logger_prefix << "AskAll true!");
      ROS_INFO_STREAM(logger_prefix << "Best Answer is: " << multi_answer.at(0).instantiation);
   }

   ros::Duration(1.0).sleep();

   // first tell
   KnowrobQuery single_tell = kc.getDefaultQueryMessage();
   single_tell.queryString = std::string("test:hasSon(a,b)");
   
   ROS_INFO_STREAM(logger_prefix << "Sending Tell Query!");
   result = kc.tell(single_tell);

   if(!result)
   {
      ROS_ERROR_STREAM(logger_prefix << "Tell false!");
   }
   else
   {
      ROS_INFO_STREAM(logger_prefix << "Tell true!");
   }


   // second ask
   multi_answer.clear();
   result = kc.askAll(single_query, multi_answer);

   if(!result)
   {
      ROS_ERROR_STREAM(logger_prefix << "AskAll false!");
   }
   else
   {
      ROS_INFO_STREAM(logger_prefix << "AskAll true!");
      ROS_INFO_STREAM(logger_prefix << "Best Answer is: " << kc.getAnswerText(multi_answer.at(0)) );
   }


   
   exit(0);
}