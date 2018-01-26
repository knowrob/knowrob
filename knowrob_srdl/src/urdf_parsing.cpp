// TODO: add license header
#include <string>
#include <vector>
#include "rosprolog.h"
#include <urdf/model.h>

/**************************************/
/********** INIT URDF *****************/
/**************************************/

urdf::Model *robot_model = new urdf::Model();

urdf::Model* get_robot_model() {
    if (robot_model == NULL)
//        throw std::runtime_error("No URDF model has been loaded, yet. You have to call load_urdf(), first.");
        throw std::runtime_error("Encountered unexpected NULL pointer when accessing URDF model, please report it.");
    return robot_model;
}

PREDICATE(load_urdf, 1) {
    std::string filename((char*)PL_A1);
    return get_robot_model()->initFile(filename);
}

/**************************************/
/******** LINK PROPERTIES *************/
/**************************************/

PREDICATE(root_link_name, 1) {
    try {
        PL_A1 = get_robot_model()->root_link_->name.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_names, 1) {
    try {
        PlTail names(PL_A1);
        for (auto const& link_entry: get_robot_model()->links_)
            names.append(link_entry.first.c_str());
        return names.close();
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}
urdf::LinkConstSharedPtr get_link(const std::string& link_name) {
    urdf::LinkConstSharedPtr link = get_robot_model()->getLink(link_name);
    if (!link)
        throw std::runtime_error("No link with name '" + link_name + "' in loaded robot.");
    return link;
}

PREDICATE(link_parent_joint, 2) {
    try {
        std::string link_name((char*) PL_A1);
        urdf::LinkConstSharedPtr link = get_link(link_name);
        if (link->parent_joint) {
            PL_A2 = link->parent_joint->name.c_str();
            return true;
        } else
            return false;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_child_joints, 2) {
    try {
        std::string link_name((char*) PL_A1);
        urdf::LinkConstSharedPtr link = get_link(link_name);
        PlTail child_joints(PL_A2);
        for (auto const& child_joint: link->child_joints)
            child_joints.append(child_joint->name.c_str());
        return child_joints.close();
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}


/**************************************/
/******** JOINT PROPERTIES ************/
/**************************************/

PREDICATE(joint_names, 1) {
    try {
        PlTail names(PL_A1);
        for (auto const& joint_entry: get_robot_model()->joints_)
            names.append(joint_entry.first.c_str());
        return names.close();
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

urdf::JointConstSharedPtr get_joint(const std::string& joint_name) {
    urdf::JointConstSharedPtr joint = get_robot_model()->getJoint(joint_name);
    if (!joint)
        throw std::runtime_error("No joint with name '" + joint_name + "' in parsed URDF.");
    return joint;
}

PREDICATE(joint_type, 2) {
    try {
        std::string joint_name((char*) PL_A1);
        switch (get_joint(joint_name)->type) {
            case (urdf::Joint::REVOLUTE): {
                PL_A2 = "revolute";
                return true;
            }
            case (urdf::Joint::PRISMATIC): {
                PL_A2 = "prismatic";
                return true;
            }
            case (urdf::Joint::CONTINUOUS): {
                PL_A2 = "continuous";
                return true;
            }
            case (urdf::Joint::FIXED): {
                PL_A2 = "fixed";
                return true;
            }
            case (urdf::Joint::PLANAR): {
                PL_A2 = "planar";
                return true;
            }
            case (urdf::Joint::FLOATING): {
                PL_A2 = "floating";
                return true;
            }
            case (urdf::Joint::UNKNOWN): {
                PL_A2 = "unknown";
                return true;
            }
            default:
                throw std::runtime_error("Found undefined joint type for joint'" + joint_name + "'.");
        }
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}
PREDICATE(joint_child_link, 2) {
    try {
        std::string joint_name((char*) PL_A1);
        PL_A2 = get_joint(joint_name)->child_link_name.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_parent_link, 2) {
    try {
        std::string joint_name((char*) PL_A1);
        PL_A2 = get_joint(joint_name)->parent_link_name.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_axis, 2) {
     try {

         std::string joint_name((char*) PL_A1);
         urdf::JointConstSharedPtr joint = get_joint(joint_name);
         // joint axis not defined for the following three joint types
         if (joint->type == urdf::Joint::FIXED ||
                 joint->type == urdf::Joint::UNKNOWN ||
                 joint->type == urdf::Joint::FLOATING)
             return false;
         PlTail l(PL_A2);
         l.append(joint->axis.x);
         l.append(joint->axis.y);
         l.append(joint->axis.z);
         return l.close();
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

// TODO: read joint axis

// TODO: read joint origin

// TODO: read joint lower pos limit

// TODO: read joint upper pos limit

// TODO: read joint vel limit

// TODO: read joint effort limit

/**************************************/
/******** DUMMY PREDICATES ************/
/**************************************/

PREDICATE(foo, 1) {
    PL_A1 = std::string("foo").c_str();
    return true;
}

PREDICATE(bar, 1) {
    PL_A1 = std::string("bar").c_str();
    return true;
}

PREDICATE_NONDET(foo_bar, 1) {
//    switch( PL_foreign_control(handle) ) {
//
//    }
    return false;
}
