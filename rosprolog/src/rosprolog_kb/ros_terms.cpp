#include <rosprolog/rosprolog_kb/rosprolog_kb.h>

#include <ros/ros.h>

namespace rosprolog_kb {
    void term_to_color(const PlTerm &rosterm, std_msgs::ColorRGBA &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.r = (double)e;
        list.next(e); value.g = (double)e;
        list.next(e); value.b = (double)e;
        list.next(e); value.a = (double)e;
    }
    
    void term_to_vector3(const PlTerm &rosterm, geometry_msgs::Vector3 &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.x = (double)e;
        list.next(e); value.y = (double)e;
        list.next(e); value.z = (double)e;
    }
    
    void term_to_point(const PlTerm &rosterm, geometry_msgs::Point &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.x = (double)e;
        list.next(e); value.y = (double)e;
        list.next(e); value.z = (double)e;
    }
    
    void term_to_quaternion(const PlTerm &rosterm, geometry_msgs::Quaternion &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.x = (double)e;
        list.next(e); value.y = (double)e;
        list.next(e); value.z = (double)e;
        list.next(e); value.w = (double)e;
    }
    
    void term_to_transform_stamped(const PlTerm &rosterm, geometry_msgs::TransformStamped &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.header.frame_id = (char*)e;
        list.next(e); value.child_frame_id = (char*)e;
        list.next(e); term_to_vector3(e, value.transform.translation);
        list.next(e); term_to_quaternion(e, value.transform.rotation);
    }
    
    void term_to_pose_stamped(const PlTerm &rosterm, geometry_msgs::PoseStamped &value) {
        PlTail list(rosterm); PlTerm e;
        list.next(e); value.header.frame_id = (char*)e;
        list.next(e); // unused
        list.next(e); term_to_point(e, value.pose.position);
        list.next(e); term_to_quaternion(e, value.pose.orientation);
    }
};
