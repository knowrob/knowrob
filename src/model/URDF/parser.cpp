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
#include <urdf/model.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

/**************************************/
/********** CONVERSIONS ***************/
/**************************************/

PlTerm to_prolog_pose(const std::string &frame, const urdf::Pose& p) {
	//
	PlTerm pos;
	PlTail posl(pos);
	posl.append(p.position.x);
	posl.append(p.position.y);
	posl.append(p.position.z);
	posl.close();
	//
	PlTerm rot;
	PlTail rotl(rot);
	rotl.append(p.rotation.x);
	rotl.append(p.rotation.y);
	rotl.append(p.rotation.z);
	rotl.append(p.rotation.w);
	rotl.close();
	//
	PlTerm pose;
	PlTail l(pose);
	l.append(frame.c_str());
	l.append(pos);
	l.append(rot);
	return pose;
}

PlCompound to_prolog_geometry(urdf::GeometryConstSharedPtr geom) {
    if (!geom)
        throw PlException(PlCompound("urdf_error", PlTerm("null_geometry")));

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
            throw PlException(PlCompound("urdf_error",
            		PlTerm("unsupported_geometry")));
    }
}

/**************************************/
/********** INIT URDF *****************/
/**************************************/

std::map<std::string,urdf::Model> robot_models;
std::mutex robot_models_mtx;

urdf::Model& get_robot_model(const char *id) {
    std::unique_lock<std::mutex> lock(robot_models_mtx);
    return robot_models[std::string(id)];
}

urdf::LinkConstSharedPtr get_link(const char* urdf_id, const char* link_name) {
    urdf::LinkConstSharedPtr link = get_robot_model(urdf_id).getLink(std::string(link_name));
    if (!link)
        throw PlException(PlCompound("urdf_error",
        		PlCompound("no_such_link", PlTerm(link_name))));
    return link;
}

urdf::JointConstSharedPtr get_joint(const char* urdf_id, const char* joint_name) {
    urdf::JointConstSharedPtr joint = get_robot_model(urdf_id).getJoint(std::string(joint_name));
    if (!joint)
        throw PlException(PlCompound("urdf_error",
        		PlCompound("no_such_joint", PlTerm(joint_name))));
    return joint;
}

bool link_has_visual_with_index(const urdf::LinkConstSharedPtr link, long index) {
    return link && (index >= 0) && (index < link->visual_array.size()) &&
            (link->visual_array[index]) &&
			//(link->visual_array[index]->name.compare("") != 0) &&
			(link->visual_array[index]->geometry);
}

bool link_has_collision_with_index(const urdf::LinkConstSharedPtr link, long index) {
    return link && (index >= 0) && (index < link->collision_array.size()) &&
            (link->collision_array[index]) &&
			//(link->collision_array[index]->name.compare("") != 0) &&
			(link->collision_array[index]->geometry);
}

bool joint_has_pos_limits(urdf::JointConstSharedPtr joint) {
    return joint->type == urdf::Joint::PRISMATIC ||
           joint->type == urdf::Joint::REVOLUTE;
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

const char* get_joint_type(urdf::JointConstSharedPtr joint) {
	switch (joint->type) {
		case urdf::Joint::REVOLUTE:   return "revolute";
		case urdf::Joint::PRISMATIC:  return "prismatic";
		case urdf::Joint::CONTINUOUS: return "continuous";
		case urdf::Joint::FIXED:      return "fixed";
		case urdf::Joint::PLANAR:     return "planar";
		case urdf::Joint::FLOATING:   return "floating";
		default:                      return "unknown";
	}
}

/**************************************/
/******** PROLOG PREDICATES ***********/
/**************************************/

// urdf_load_file(Object, File)
PREDICATE(urdf_load_file, 2) {
	std::string filename((char*)PL_A2);
	if(get_robot_model(PL_A1).initFile(filename)) {
		return true;
	} else {
		std::unique_lock<std::mutex> lock(robot_models_mtx);
		std::string urdf_id((char*)PL_A1);
		robot_models.erase(urdf_id);
		return false;
	}
}

// urdf_unload_file(Object)
PREDICATE(urdf_unload_file, 1) {
	std::string urdf_id((char*)PL_A1);
	std::unique_lock<std::mutex> lock(robot_models_mtx);
	robot_models.erase(urdf_id);
	return true;
}

//PREDICATE(load_urdf_param, 2) {
//    std::string urdf_id((char*)PL_A1);
//    std::string param_name((char*)PL_A2);
//    return get_robot_model(urdf_id).initParam(param_name);
//}

//PREDICATE(load_urdf_string, 2) {
//    std::string urdf_id((char*)PL_A1);
//    std::string urdf_string((char*)PL_A2);
//    return get_robot_model(urdf_id).initString(urdf_string);
//}

// urdf_robot_name(Object, Name)
PREDICATE(urdf_robot_name, 2) {
	PL_A2 = get_robot_model(PL_A1).name_.c_str();
	return true;
}

// urdf_root_link(Object, RootLink)
PREDICATE(urdf_root_link, 2) {
	PL_A2 = get_robot_model(PL_A1).root_link_->name.c_str();
	return true;
}

// urdf_link_parent_joint(Object, Link, Parent)
PREDICATE(urdf_link_parent_joint, 3) {
	urdf::LinkConstSharedPtr link = get_link(PL_A1,PL_A2);
	if (link->parent_joint) {
		PL_A3 = link->parent_joint->name.c_str();
		return true;
	}
	else {
		return false;
	}
}

// urdf_link_child_joints(Object, Link, Children)
PREDICATE(urdf_link_child_joints, 3) {
	urdf::LinkConstSharedPtr link = get_link(PL_A1,PL_A2);
    PlTail child_joints(PL_A3);
    for (auto const& child_joint: link->child_joints) {
    	child_joints.append(child_joint->name.c_str());
    }
    return child_joints.close();
}

// urdf_link_inertial(Object, Link, Inertia, Origin, Mass)
PREDICATE(urdf_link_inertial, 5) {
	urdf::LinkConstSharedPtr link = get_link(PL_A1,PL_A2);
	if(link->inertial) {
		PlTail inertia(PL_A3);
		inertia.append(link->inertial->ixx);
		inertia.append(link->inertial->ixy);
		inertia.append(link->inertial->ixz);
		inertia.append(link->inertial->iyy);
		inertia.append(link->inertial->iyz);
		inertia.append(link->inertial->izz);
		inertia.close();
		PL_A4 = to_prolog_pose(link->name, link->inertial->origin);
		PL_A5 = link->inertial->mass;
		return true;
	}
	else {
		return false;
	}
}

//PREDICATE(urdf_link_material, 4) {
//	urdf::LinkConstSharedPtr link = get_link(PL_A1,PL_A2);
//        long index = (long) PL_A3;
//        urdf::LinkConstSharedPtr link = get_link(urdf_id,link_name);
//        if (!link_has_visual_with_index(link, index) ||
//                !link->visual_array[index]->material ||
//                link->visual_array[index]->material->texture_filename.compare("") == 0)
//            return false;
//        PL_A4 = link->visual_array[index]->material->texture_filename.c_str();
//PlTermv rgba_term(PL_A4);
//rgba_term[0] = link->visual_array[index]->material->color.r;
//rgba_term[1] = link->visual_array[index]->material->color.g;
//rgba_term[2] = link->visual_array[index]->material->color.b;
//rgba_term[3] = link->visual_array[index]->material->color.a;
//        PL_A4 = link->visual_array[index]->material->name.c_str();
//        return true;
//}

// urdf_link_num_visuals(Object,Link,Count)
PREDICATE(urdf_link_num_visuals, 3) {
	urdf::LinkConstSharedPtr link = get_link(PL_A1,PL_A2);
	PL_A3 = (long) link->visual_array.size();
	return true;
}

// urdf_link_nth_visual_shape(Object,Link,Index,ShapeTerm,Origin)
PREDICATE(urdf_link_nth_visual_shape, 5) {
	urdf::LinkConstSharedPtr link = get_link(PL_A1,PL_A2);
	long index = (long) PL_A3;
    if (link_has_visual_with_index(link, index)) {
    	//link->visual_array[index]->name.c_str();
    	PL_A4 = to_prolog_geometry(link->visual_array[index]->geometry);
    	PL_A5 = to_prolog_pose(link->name,link->visual_array[index]->origin);
        return true;
    }
	else {
		return false;
	}
}

// urdf_link_num_collisions(Object,Link,Count)
PREDICATE(urdf_link_num_collisions, 3) {
	urdf::LinkConstSharedPtr link = get_link(PL_A1,PL_A2);
	PL_A3 = (long) link->collision_array.size();
	return true;
}

// urdf_link_nth_collision_shape(Object,Link,Index,ShapeTerm,Origin)
PREDICATE(urdf_link_nth_collision_shape, 5) {
	urdf::LinkConstSharedPtr link = get_link(PL_A1,PL_A2);
	long index = (long) PL_A3;
    if (link_has_collision_with_index(link, index)) {
    	//link->collision_array[index]->name.c_str();
    	PL_A4 = to_prolog_geometry(link->collision_array[index]->geometry);
    	PL_A5 = to_prolog_pose(link->name,link->collision_array[index]->origin);
        return true;
    }
	else {
		return false;
	}
}

// urdf_joint_type(Object, Joint, Type)
PREDICATE(urdf_joint_type, 3) {
    PL_A3 = get_joint_type(get_joint(PL_A1,PL_A2));
    return true;
}

// urdf_joint_child_link(Object, Joint, Child)
PREDICATE(urdf_joint_child_link, 3) {
	PL_A3 = get_joint(PL_A1,PL_A2)->child_link_name.c_str();
	return true;
}

// urdf_joint_parent_link(Object, Joint, Parent)
PREDICATE(urdf_joint_parent_link, 3) {
	PL_A3 = get_joint(PL_A1,PL_A2)->parent_link_name.c_str();
	return true;
}

// urdf_joint_axis(Object, Joint, [X,Y,Z])
PREDICATE(urdf_joint_axis, 3) {
	urdf::JointConstSharedPtr joint = get_joint(PL_A1,PL_A2);
	// joint axis not defined for the following three joint types
	if (joint->type == urdf::Joint::FIXED ||
			joint->type == urdf::Joint::UNKNOWN ||
			joint->type == urdf::Joint::FLOATING) {
        return false;
	}
	PlTail l(PL_A3);
	l.append(joint->axis.x);
	l.append(joint->axis.y);
	l.append(joint->axis.z);
	return l.close();
}

// urdf_joint_origin(Object, Joint, [Frame,Position,Rotation])
// This is the transform from the parent link to the child link.
// The joint is located at the origin of the child link
PREDICATE(urdf_joint_origin, 3) {
	urdf::JointConstSharedPtr joint = get_joint(PL_A1,PL_A2);
	PL_A3 = to_prolog_pose(
			joint->parent_link_name,
			joint->parent_to_joint_origin_transform);
	return true;
}

// urdf_joint_calibration(Object, Joint, Falling)
PREDICATE(urdf_joint_calibration_falling, 3) {
	urdf::JointConstSharedPtr joint = get_joint(PL_A1,PL_A2);
	if (joint->calibration && joint->calibration->falling) {
		PL_A3 = *(joint->calibration->falling);
		return true;
	}
	else {
		return false;
	}
}

// urdf_joint_calibration(Object, Joint, Rising)
PREDICATE(urdf_joint_calibration_rising, 3) {
	urdf::JointConstSharedPtr joint = get_joint(PL_A1,PL_A2);
	if (joint->calibration && joint->calibration->rising) {
		PL_A3 = *(joint->calibration->rising);
		return true;
	}
	else {
		return false;
	}
}

// urdf_joint_damping(Object, Joint, Damping)
PREDICATE(urdf_joint_damping, 3) {
	urdf::JointConstSharedPtr joint = get_joint(PL_A1,PL_A2);
	if (joint->dynamics){
		PL_A3 = joint->dynamics->damping;
		return true;
	}
	else {
		return false;
	}
}

// urdf_joint_friction(Object, Joint, Friction)
PREDICATE(urdf_joint_friction, 3) {
	urdf::JointConstSharedPtr joint = get_joint(PL_A1,PL_A2);
	if (joint->dynamics){
		PL_A3 = joint->dynamics->friction;
		return true;
	}
	else {
		return false;
	}
}

// urdf_joint_hard_limits(Object, Joint, [LL,UL], VelMax, EffMax)
PREDICATE(urdf_joint_hard_limits, 5) {
	urdf::JointConstSharedPtr joint = get_joint(PL_A1,PL_A2);
	if (joint->limits) {
        PlTail l(PL_A3);
        if(joint_has_pos_limits(joint)) {
            l.append(joint->limits->lower);
            l.append(joint->limits->upper);
        }
        else {
    		return false;
        }
        l.close();
        PL_A4 = (joint_has_vel_limit(joint) ? joint->limits->velocity : 0.0);
        PL_A5 = (joint_has_effort_limit(joint) ? joint->limits->effort : 0.0);
		return true;
	}
	else {
		return false;
	}
}

// urdf_joint_soft_limits(Object, Joint, [LL,UL], KP, KV)
PREDICATE(urdf_joint_soft_limits, 5) {
	urdf::JointConstSharedPtr joint = get_joint(PL_A1,PL_A2);
	if (joint->safety) {
		PlTail l(PL_A3);
		l.append(joint->safety->soft_lower_limit);
		l.append(joint->safety->soft_upper_limit);
		l.close();
		PL_A4 = joint->safety->k_position;
		PL_A5 = joint->safety->k_velocity;
		return true;
	}
	else {
		return false;
	}
}
