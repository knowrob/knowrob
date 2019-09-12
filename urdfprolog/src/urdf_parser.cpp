/*
  Copyright (C) 2018 Georg Bartels
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include "rosprolog.h"
#include <urdf/model.h>

/**************************************/
/********** CONVERSIONS ***************/
/**************************************/

void to_prolog_position(PlTermv &out, const urdf::Vector3 &v) {
    PlTail l(out[0]);
    l.append(v.x);
    l.append(v.y);
    l.append(v.z);
    l.close();
}

void to_prolog_orientation(PlTermv &out, const urdf::Rotation &q) {
    PlTail l(out[1]);
    l.append(q.x);
    l.append(q.y);
    l.append(q.z);
    l.append(q.w);
    l.close();
}

PlCompound to_prolog_pose(const urdf::Pose& p) {
    // create term `pose([X,Y,Z], [QX,QY,QZ,QW])`
    PlTermv prolog_pose(2);
    to_prolog_position(prolog_pose, p.position);
    to_prolog_orientation(prolog_pose, p.rotation);
    return PlCompound("pose", prolog_pose);
}

PlCompound to_prolog_geometry(urdf::GeometryConstSharedPtr geom) {
    if (!geom)
        throw std::runtime_error("Unexcepted null-ptr for geometry.");

    switch(geom->type) {
        case urdf::Geometry::BOX: {
            urdf::BoxConstSharedPtr box = urdf::dynamic_pointer_cast<const urdf::Box>(geom);
            PlTermv dim_term(3);
            dim_term[0] = box->dim.x;
            dim_term[1] = box->dim.y;
            dim_term[2] = box->dim.z;
            return PlCompound("box", dim_term);
        }
        case urdf::Geometry::SPHERE: {
            urdf::SphereConstSharedPtr sphere = urdf::dynamic_pointer_cast<const urdf::Sphere>(geom);
            PlTermv dim_term(1);
            dim_term[0] = sphere->radius;
            return PlCompound("sphere", dim_term);
        }
        case urdf::Geometry::CYLINDER : {
            urdf::CylinderConstSharedPtr cylinder = urdf::dynamic_pointer_cast<const urdf::Cylinder>(geom);
            PlTermv dim_term(2);
            dim_term[0] = cylinder->radius;
            dim_term[1] = cylinder->length;
            return PlCompound("cylinder", dim_term);
        }
        case urdf::Geometry::MESH : {
            urdf::MeshConstSharedPtr mesh = urdf::dynamic_pointer_cast<const urdf::Mesh>(geom);
            PlTermv dim_term(2);
            dim_term[0] = mesh->filename.c_str();
            PlTail scale(dim_term[1]);
            scale.append(mesh->scale.x);
            scale.append(mesh->scale.y);
            scale.append(mesh->scale.z);
            scale.close();
            return PlCompound("mesh", dim_term);
        }
        default:
            throw std::runtime_error("Encountered unsupported geometry type.");
    }
}

PlCompound to_prolog_rgba(const urdf::Color& rgba) {
    PlTermv rgba_term(4);
    rgba_term[0] = rgba.r;
    rgba_term[1] = rgba.g;
    rgba_term[2] = rgba.b;
    rgba_term[3] = rgba.a;
    return PlCompound("rgba", rgba_term);
}

/**************************************/
/********** INIT URDF *****************/
/**************************************/

std::map<std::string,urdf::Model> robot_models;
std::mutex robot_models_mtx;

urdf::Model& get_robot_model(const std::string &id) {
    std::unique_lock<std::mutex> lock(robot_models_mtx);
    return robot_models[id];
}

PREDICATE(load_urdf_file, 2) {
    std::string urdf_id((char*)PL_A1);
    std::string filename((char*)PL_A2);
    if(get_robot_model(urdf_id).initFile(filename)) {
        return true;
    } else {
        std::unique_lock<std::mutex> lock(robot_models_mtx);
        robot_models.erase(urdf_id);
        return false;
    }
}

PREDICATE(unload_urdf_file, 1) {
    std::string urdf_id((char*)PL_A1);
    std::unique_lock<std::mutex> lock(robot_models_mtx);
    robot_models.erase(urdf_id);
    return true;
}

PREDICATE(load_urdf_param, 2) {
    std::string urdf_id((char*)PL_A1);
    std::string param_name((char*)PL_A2);
    return get_robot_model(urdf_id).initParam(param_name);
}

PREDICATE(load_urdf_string, 2) {
    std::string urdf_id((char*)PL_A1);
    std::string urdf_string((char*)PL_A2);
    return get_robot_model(urdf_id).initString(urdf_string);
}

/**************************************/
/******** ROBOT PROPERTIES ************/
/**************************************/

PREDICATE(robot_name, 2) {
    try {
	std::string urdf_id((char*)PL_A1);
        PL_A2 = get_robot_model(urdf_id).name_.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

/**************************************/
/******** LINK PROPERTIES *************/
/**************************************/

urdf::LinkConstSharedPtr get_link(const std::string& urdf_id, const std::string& link_name) {
    urdf::LinkConstSharedPtr link = get_robot_model(urdf_id).getLink(link_name);
    if (!link)
        throw std::runtime_error("No link with name '" + link_name + "' in loaded robot.");
    return link;
}

bool link_has_visual_with_index(const urdf::LinkConstSharedPtr link, long index) {
    return link && (index >= 0) && (index < link->visual_array.size()) &&
            (link->visual_array[index]);
}

bool link_has_collision_with_index(const urdf::LinkConstSharedPtr link, long index) {
    return link && (index >= 0) && (index < link->collision_array.size()) &&
            (link->collision_array[index]);
}

PREDICATE(root_link_name, 2) {
    try {
	std::string urdf_id((char*)PL_A1);
        PL_A2 = get_robot_model(urdf_id).root_link_->name.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_names, 2) {
    try {
	std::string urdf_id((char*)PL_A1);
        PlTail names(PL_A2);
        for (auto const& link_entry: get_robot_model(urdf_id).links_)
            names.append(link_entry.first.c_str());
        return names.close();
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_parent_joint, 3) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (link->parent_joint) {
            PL_A3 = link->parent_joint->name.c_str();
            return true;
        } else
            return false;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_child_joints, 3) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        PlTail child_joints(PL_A3);
        for (auto const& child_joint: link->child_joints)
            child_joints.append(child_joint->name.c_str());
        return child_joints.close();
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_inertial_origin, 3) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link->inertial)
            return false;
        PL_A3 = to_prolog_pose(link->inertial->origin);
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_inertial_mass, 3) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link->inertial)
            return false;
        PL_A3 = link->inertial->mass;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_inertial_inertia, 3) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link->inertial)
            return false;
        PlTail inertia(PL_A3);
        inertia.append(link->inertial->ixx);
        inertia.append(link->inertial->ixy);
        inertia.append(link->inertial->ixz);
        inertia.append(link->inertial->iyy);
        inertia.append(link->inertial->iyz);
        inertia.append(link->inertial->izz);
        return inertia.close();
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_num_visuals, 3) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        PL_A3 = (long) link->visual_array.size();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_visual_type, 4) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        long visual_index = (long) PL_A3;
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link_has_visual_with_index(link, visual_index) ||
                !link->visual_array[visual_index]->geometry)
            return false;
        switch (link->visual_array[visual_index]->geometry->type) {
            case urdf::Geometry::BOX: {
                PL_A4 = "box";
                return true;
            }
            case urdf::Geometry::CYLINDER: {
                PL_A4 = "cylinder";
                return true;
            }
            case urdf::Geometry::SPHERE: {
                PL_A4 = "sphere";
                return true;
            }
            case urdf::Geometry::MESH: {
                PL_A4 = "mesh";
                return true;
            }
            default:
                return false;
        }
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_visual_name, 4) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        long index = (long) PL_A3;
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link_has_visual_with_index(link, index) ||
                (link->visual_array[index]->name.compare("") == 0))
            return false;
        PL_A4 = link->visual_array[index]->name.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_visual_origin, 4) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        long index = (long) PL_A3;
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link_has_visual_with_index(link, index))
            return false;
        PL_A4 = to_prolog_pose(link->visual_array[index]->origin);
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_visual_geometry, 4) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        long index = (long) PL_A3;
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link_has_visual_with_index(link, index) ||
                !link->visual_array[index]->geometry)
            return false;
        PL_A4 = to_prolog_geometry(link->visual_array[index]->geometry);
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_material_name, 4) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        long index = (long) PL_A3;
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link_has_visual_with_index(link, index) ||
                !link->visual_array[index]->material)
            return false;
        PL_A4 = link->visual_array[index]->material->name.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_material_color, 4) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        long index = (long) PL_A3;
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link_has_visual_with_index(link, index) ||
                !link->visual_array[index]->material)
            return false;
        PL_A4 = to_prolog_rgba(link->visual_array[index]->material->color);
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_material_texture, 4) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        long index = (long) PL_A3;
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link_has_visual_with_index(link, index) ||
                !link->visual_array[index]->material ||
                link->visual_array[index]->material->texture_filename.compare("") == 0)
            return false;
        PL_A4 = link->visual_array[index]->material->texture_filename.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_num_collisions, 3) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        PL_A3 = (long) link->collision_array.size();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_collision_type, 4) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        long index = (long) PL_A3;
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link_has_collision_with_index(link, index) ||
                !link->collision_array[index]->geometry)
            return false;
        switch (link->collision_array[index]->geometry->type) {
            case urdf::Geometry::BOX: {
                PL_A4 = "box";
                return true;
            }
            case urdf::Geometry::CYLINDER: {
                PL_A4 = "cylinder";
                return true;
            }
            case urdf::Geometry::SPHERE: {
                PL_A4 = "sphere";
                return true;
            }
            case urdf::Geometry::MESH: {
                PL_A4 = "mesh";
                return true;
            }
            default:
                return false;
        }
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_collision_name, 4) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        long index = (long) PL_A3;
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link_has_collision_with_index(link, index) ||
                (link->collision_array[index]->name.compare("") == 0))
            return false;
        PL_A4 = link->collision_array[index]->name.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_collision_origin, 4) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        long index = (long) PL_A3;
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link_has_collision_with_index(link, index))
            return false;
        PL_A4 = to_prolog_pose(link->collision_array[index]->origin);
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(link_collision_geometry, 4) {
    try {
	std::string urdf_id((char*)PL_A1);
        std::string link_name((char*) PL_A2);
        long index = (long) PL_A3;
        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
        if (!link_has_collision_with_index(link, index) ||
                !link->collision_array[index]->geometry)
            return false;
        PL_A4 = to_prolog_geometry(link->collision_array[index]->geometry);
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

/**************************************/
/******** JOINT PROPERTIES ************/
/**************************************/

urdf::JointConstSharedPtr get_joint(const std::string& urdf_id, const std::string& joint_name) {
    urdf::JointConstSharedPtr joint = get_robot_model(urdf_id).getJoint(joint_name);
    if (!joint)
        throw std::runtime_error("No joint with name '" + joint_name + "' in parsed URDF.");
    return joint;
}

bool joint_has_pos_limits(urdf::JointConstSharedPtr joint) {
    return joint->type == urdf::Joint::PRISMATIC || joint->type == urdf::Joint::REVOLUTE;
}

bool joint_has_vel_limit(urdf::JointConstSharedPtr joint) {
    return joint->type == urdf::Joint::REVOLUTE ||
           joint->type == urdf::Joint::PRISMATIC ||
           joint->type == urdf::Joint::CONTINUOUS;
}

bool joint_has_effort_limit(urdf::JointConstSharedPtr joint) {
    // joints that have velocity limits should also have effort limits
    return joint_has_vel_limit(joint);
}


PREDICATE(joint_names, 2) {
    try {
        std::string urdf_id((char*)PL_A1);
        PlTail names(PL_A2);
        for (auto const& joint_entry: get_robot_model(urdf_id).joints_)
            names.append(joint_entry.first.c_str());
        return names.close();
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_type, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        switch (get_joint(urdf_id,joint_name)->type) {
            case (urdf::Joint::REVOLUTE): {
                PL_A3 = "revolute";
                return true;
            }
            case (urdf::Joint::PRISMATIC): {
                PL_A3 = "prismatic";
                return true;
            }
            case (urdf::Joint::CONTINUOUS): {
                PL_A3 = "continuous";
                return true;
            }
            case (urdf::Joint::FIXED): {
                PL_A3 = "fixed";
                return true;
            }
            case (urdf::Joint::PLANAR): {
                PL_A3 = "planar";
                return true;
            }
            case (urdf::Joint::FLOATING): {
                PL_A3 = "floating";
                return true;
            }
            case (urdf::Joint::UNKNOWN): {
                PL_A3 = "unknown";
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

PREDICATE(joint_child_link, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        PL_A3 = get_joint(urdf_id,joint_name)->child_link_name.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_parent_link, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        PL_A3 = get_joint(urdf_id,joint_name)->parent_link_name.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_axis, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        // joint axis not defined for the following three joint types
        if (joint->type == urdf::Joint::FIXED ||
            joint->type == urdf::Joint::UNKNOWN ||
            joint->type == urdf::Joint::FLOATING)
            return false;
        PlTail l(PL_A3);
        l.append(joint->axis.x);
        l.append(joint->axis.y);
        l.append(joint->axis.z);
        return l.close();
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_origin, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        PL_A3 = to_prolog_pose(joint->parent_to_joint_origin_transform);
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_lower_pos_limit, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!joint_has_pos_limits(joint))
            return false;
        PL_A3 = joint->limits->lower;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_upper_pos_limit, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!joint_has_pos_limits(joint))
            return false;
        PL_A3 = joint->limits->upper;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_velocity_limit, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!joint_has_vel_limit(joint))
            return false;
        PL_A3 = joint->limits->velocity;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_effort_limit, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!joint_has_effort_limit(joint))
            return false;
        PL_A3 = joint->limits->effort;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_calibration_rising, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!(joint->calibration && joint->calibration->rising))
            return false;
        PL_A3 = *(joint->calibration->rising);
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_calibration_falling, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!(joint->calibration && joint->calibration->falling))
            return false;
        PL_A3 = *(joint->calibration->falling);
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_dynamics_damping, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!(joint->dynamics))
            return false;
        PL_A3 = joint->dynamics->damping;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_dynamics_friction, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!(joint->dynamics))
            return false;
        PL_A3 = joint->dynamics->friction;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_mimic_joint_name, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!(joint->mimic))
            return false;
        PL_A3 = joint->mimic->joint_name.c_str();
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_mimic_multiplier, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!(joint->mimic))
            return false;
        PL_A3 = joint->mimic->multiplier;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_mimic_offset, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!(joint->mimic))
            return false;
        PL_A3 = joint->mimic->offset;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_safety_lower_limit, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!(joint->safety))
            return false;
        PL_A3 = joint->safety->soft_lower_limit;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_safety_upper_limit, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!(joint->safety))
            return false;
        PL_A3 = joint->safety->soft_upper_limit;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_safety_kp, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!(joint->safety))
            return false;
        PL_A3 = joint->safety->k_position;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}

PREDICATE(joint_safety_kv, 3) {
    try {
        std::string urdf_id((char*)PL_A1);
        std::string joint_name((char*) PL_A2);
        urdf::JointConstSharedPtr joint = get_joint(urdf_id,joint_name);
        if (!(joint->safety))
            return false;
        PL_A3 = joint->safety->k_velocity;
        return true;
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
    }
}
