#include "rosprolog/rosprolog_kb/rosprolog_kb.h"
#include "private/rosprolog_kb_priv.h"

#include <stdlib.h>

#include <ros/ros.h>
#include <ros/package.h>

/*********************************/
/********** KBNode ***************/
/*********************************/
    
rosprolog_kb::KBNode::KBNode():
	nh_(),
	thread_(&KBNode::run, this),
	thread_pool_(2)
{
}

rosprolog_kb::KBNode::~KBNode()
{
	ros::shutdown();
	thread_.join();
}

rosprolog_kb::KBNode& rosprolog_kb::KBNode::get()
{
	if(!ros::isInitialized()) {
		int argc=0;
		ros::init(argc, (char**)NULL, std::string("knowrob"));
	}
	static rosprolog_kb::KBNode the_node;
	return the_node;
}

ros::NodeHandle& rosprolog_kb::KBNode::node()
{
	return KBNode::get().nh_;
}

PrologPool& rosprolog_kb::KBNode::thread_pool()
{
	return KBNode::get().thread_pool_;
}

void rosprolog_kb::KBNode::run()
{
	ROS_DEBUG("rosprolog_kb thread started.");
	
	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
	
	ROS_DEBUG("rosprolog_kb thread stopped.");
}

ros::NodeHandle& rosprolog_kb::node()
{
	return rosprolog_kb::KBNode::node();
}

PrologPool& rosprolog_kb::thread_pool()
{
	return rosprolog_kb::KBNode::thread_pool();
}

PREDICATE(ros_init, 0) {
	rosprolog_kb::KBNode::node();
	return TRUE;
}

/*********************************/
/********** Logging **************/
/*********************************/

PREDICATE(ros_info, 1)
{
	ROS_INFO("%s", (char*)PL_A1);
	return TRUE;
}

PREDICATE(ros_warn, 1)
{
	ROS_WARN("%s", (char*)PL_A1);
	return TRUE;
}

PREDICATE(ros_error, 1)
{
	ROS_ERROR("%s", (char*)PL_A1);
	return TRUE;
}

PREDICATE(ros_debug, 1)
{
	ROS_DEBUG("%s", (char*)PL_A1);
	return TRUE;
}

/*********************************/
/******** ROS Packages ***********/
/*********************************/

PREDICATE(ros_package_path, 2)
{
	std::string path = ros::package::getPath(std::string((char*)PL_A1));
	if(path.empty()) return FALSE;
	PL_A2 = path.c_str();
	return TRUE;
}

PREDICATE(ros_package_command, 2)
{
	PL_A2 = ros::package::command(std::string((char*)PL_A1)).c_str();
	return TRUE;
}
