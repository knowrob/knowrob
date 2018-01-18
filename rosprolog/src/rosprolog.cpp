#include "rosprolog.h"

#include <stdlib.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>

#define ROSPROLOG_TF_CACHE_TIME 10.0

tf::TransformListener *listener = NULL;

PREDICATE(ros_init, 0) {
  if(!ros::isInitialized()) {
    int argc=0;
    ros::init(argc, (char**)NULL, std::string("knowrob"));
  }
  return TRUE;
}

/*********************************/
/********** Parameters ***********/
/*********************************/

PREDICATE(ros_param_get_string, 2) {
  std::string key((char*)PL_A1);
  std::string value;
  ros::param::get(key, value);
  PL_A2 = value.c_str();
  return TRUE;
}
PREDICATE(ros_param_get_int, 2) {
  std::string key((char*)PL_A1);
  int value;
  ros::param::get(key, value);
  PL_A2 = value;
  return TRUE;
}
PREDICATE(ros_param_get_double, 2) {
  std::string key((char*)PL_A1);
  double value;
  ros::param::get(key, value);
  PL_A2 = value;
  return TRUE;
}

PREDICATE(ros_param_set_string, 2) {
  std::string key((char*)PL_A1);
  ros::param::set(key,(char*)PL_A2);
  return TRUE;
}
PREDICATE(ros_param_set_double, 2) {
  std::string key((char*)PL_A1);
  ros::param::set(key,(double)PL_A2);
  return TRUE;
}
PREDICATE(ros_param_set_int, 2) {
  std::string key((char*)PL_A1);
  ros::param::set(key,(int)PL_A2);
  return TRUE;
}

/*********************************/
/********** Logging **************/
/*********************************/

PREDICATE(ros_info, 1) {
  ROS_INFO("%s\n", (char*)PL_A1);
  return TRUE;
}
PREDICATE(ros_warn, 1) {
  ROS_WARN("%s\n", (char*)PL_A1);
  return TRUE;
}
PREDICATE(ros_error, 1) {
  ROS_ERROR("%s\n", (char*)PL_A1);
  return TRUE;
}

/*********************************/
/******** ROS Packages ***********/
/*********************************/

PREDICATE(ros_package_path, 2) {
  std::string path = ros::package::getPath(std::string((char*)PL_A1));
  if(path.empty()) return FALSE;
  PL_A2 = path.c_str();
  return TRUE;
}

PREDICATE(ros_package_command, 2) {
  PL_A2 = ros::package::command(std::string((char*)PL_A1)).c_str();
  return TRUE;
}

/*********************************/
/******* TF integration **********/
/*********************************/

void tf_pl_vector(PlTermv &out, const tf::Vector3 &v) {
  PlTail l(out[0]);
  l.append(v.x());
  l.append(v.y());
  l.append(v.z());
  l.close();
}
void tf_pl_position(PlTermv &out, const geometry_msgs::Point &v) {
  PlTail l(out[0]);
  l.append(v.x);
  l.append(v.y);
  l.append(v.z);
  l.close();
}
void tf_pl_quaternion(PlTermv &out, const tf::Quaternion &q) {
  PlTail l(out[1]);
  l.append(q.x());
  l.append(q.y());
  l.append(q.z());
  l.append(q.w());
  l.close();
}
void tf_pl_quaternion(PlTermv &out, const geometry_msgs::Quaternion &q) {
  PlTail l(out[1]);
  l.append(q.x);
  l.append(q.y);
  l.append(q.z);
  l.append(q.w);
  l.close();
}

void tf_pl_to_point(const PlTerm &arg, geometry_msgs::PointStamped &p) {
  PlTail list(arg); PlTerm value;
  list.next(value); p.point.x = value;
  list.next(value); p.point.y = value;
  list.next(value); p.point.z = value;
}
void tf_point_to_pl(const geometry_msgs::PointStamped &p, const PlTerm &term) {
  PlTail l(term);
  l.append(p.point.x);
  l.append(p.point.y);
  l.append(p.point.z);
  l.close();
}

void tf_pl_to_quaternion(const PlTerm &arg, geometry_msgs::QuaternionStamped &p) {
  PlTail list(arg); PlTerm value;
  list.next(value); p.quaternion.x = value;
  list.next(value); p.quaternion.y = value;
  list.next(value); p.quaternion.z = value;
  list.next(value); p.quaternion.w = value;
}
void tf_quaternion_to_pl(const geometry_msgs::QuaternionStamped &p, const PlTerm &term) {
  PlTail l(term);
  l.append(p.quaternion.x);
  l.append(p.quaternion.y);
  l.append(p.quaternion.z);
  l.append(p.quaternion.w);
  l.close();
}

void tf_pl_to_pose(const PlTerm &arg, geometry_msgs::PoseStamped &p) {
  PlTail pos_list(arg[1]), rot_list(arg[2]); PlTerm value;
  pos_list.next(value); p.pose.position.x    = value;
  pos_list.next(value); p.pose.position.y    = value;
  pos_list.next(value); p.pose.position.z    = value;
  rot_list.next(value); p.pose.orientation.x = value;
  rot_list.next(value); p.pose.orientation.y = value;
  rot_list.next(value); p.pose.orientation.z = value;
  rot_list.next(value); p.pose.orientation.w = value;
}

tf::TransformListener* tf_listener() {
  if(listener==NULL) {
    listener = new tf::TransformListener(ros::Duration(ROSPROLOG_TF_CACHE_TIME));
  }
  return listener;
}

PREDICATE(tf_listener_start, 0) {
  tf_listener();
  return TRUE;
}

// tf_lookup_transform(TargetFrame, SourceFrame, Transform)
PREDICATE(tf_lookup_transform, 3) {
  tf::StampedTransform transform;
  try {
    tf_listener()->lookupTransform((char*)PL_A1, (char*)PL_A2, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return FALSE;
  }
  // create term `pose([X,Y,Z], [QX,QY,QZ,QW])`
  PlTermv pose(2);
  tf_pl_vector(pose, transform.getOrigin());
  tf_pl_quaternion(pose, transform.getRotation());
  PL_A3 = PlCompound("pose", pose);
  return TRUE;
}

// tf_transform_point(SourceFrame, TargetFrame, PointIn, PointOut)
PREDICATE(tf_transform_point, 4) {
  const char* source_frame = (char*)PL_A1;
  const char* target_frame = (char*)PL_A2;
  // read input point
  geometry_msgs::PointStamped in,out;
  tf_pl_to_point(PL_A3, in);
  in.header.stamp = ros::Time();
  in.header.frame_id = source_frame;
  // transform into target_frame
  try {
    tf_listener()->transformPoint(target_frame, in, out);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return FALSE;
  }
  // write point output
  tf_point_to_pl(out, PL_A4);
  return TRUE;
}

// tf_transform_quaternion(SourceFrame, TargetFrame, QuaternionIn, QuaternionOut)
PREDICATE(tf_transform_quaternion, 4) {
  const char* source_frame = (char*)PL_A1;
  const char* target_frame = (char*)PL_A2;
  // read input point
  geometry_msgs::QuaternionStamped in,out;
  tf_pl_to_quaternion(PL_A3, in);
  in.header.stamp = ros::Time();
  in.header.frame_id = source_frame;
  // transform into target_frame
  try {
    tf_listener()->transformQuaternion(target_frame, in, out);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return FALSE;
  }
  // write point output
  tf_quaternion_to_pl(out, PL_A4);
  return TRUE;
}

// tf_transform_pose(SourceFrame, TargetFrame, PoseIn, PoseOut)
PREDICATE(tf_transform_pose, 4) {
  const char* source_frame = (char*)PL_A1;
  const char* target_frame = (char*)PL_A2;
  // read input point
  geometry_msgs::PoseStamped in,out;
  tf_pl_to_pose(PL_A3, in);
  in.header.stamp = ros::Time();
  in.header.frame_id = source_frame;
  // transform into target_frame
  try {
    tf_listener()->transformPose(target_frame, in, out);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return FALSE;
  }
  // write point output
  PlTermv pose(2);
  tf_pl_position(pose, out.pose.position);
  tf_pl_quaternion(pose, out.pose.orientation);
  PL_A4 = PlCompound("pose", pose);
  return TRUE;
}
