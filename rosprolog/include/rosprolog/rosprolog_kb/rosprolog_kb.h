#ifndef __ROSPROLOG_KB_H__
#define __ROSPROLOG_KB_H__

#include <ros/ros.h>

#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <string>

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <rosprolog/rosprolog_node/PrologPool.h>

namespace rosprolog_kb {
	/**
	 * A ROS node supposed to be used to access ROS 
	 * from within the KB.
	 **/
	ros::NodeHandle& node();
	
	/**
	 * A pool of Prolog engines to issue queries
	 * in C++ code.
	 **/
	PrologPool& thread_pool();
	
	void term_to_color(const PlTerm &term, std_msgs::ColorRGBA &value);
	
	void term_to_vector3(const PlTerm &term, geometry_msgs::Vector3 &value);
	
	void term_to_point(const PlTerm &term, geometry_msgs::Point &value);
	
	void term_to_quaternion(const PlTerm &term, geometry_msgs::Quaternion &value);
	
	void term_to_pose_stamped(const PlTerm &term, geometry_msgs::PoseStamped &value);
	
	void term_to_transform_stamped(const PlTerm &term, geometry_msgs::TransformStamped &value);
};

#endif
