#ifndef __ROS_PROLOG_H__

#define __ROS_PROLOG_H__

#include <ros/ros.h>

#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <stdio.h>
#include <dlfcn.h>
#include <iostream>
#include <string>
#include <memory>

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace std_msgs {
    void pl_term_color(const PlTerm &pl_term, ColorRGBA &value);
};
namespace geometry_msgs {
    void pl_term_vector3(const PlTerm &pl_term, Vector3 &value);
    void pl_term_point(const PlTerm &pl_term, Point &value);
    void pl_term_quaternion(const PlTerm &pl_term, Quaternion &value);
    void pl_term_pose_stamped(const PlTerm &pl_term, PoseStamped &value);
    void pl_term_transform_stamped(const PlTerm &pl_term, TransformStamped &value);
};

#endif

