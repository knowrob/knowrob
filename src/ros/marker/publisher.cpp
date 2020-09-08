#include <knowrob/ros/marker/publisher.h>

MarkerPublisher::MarkerPublisher(ros::NodeHandle &node) :
		pub_(node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 5)),
		idCounter_(0)
{
	msg_.ns = "belief_state";
	msg_.frame_locked = true;
	msg_.mesh_use_embedded_materials = true;
	msg_.lifetime = ros::Duration();
}

void MarkerPublisher::publish(visualization_msgs::MarkerArray &array_msg)
{
	pub_.publish(array_msg);
}

int MarkerPublisher::getID(const std::string &name)
{
	std::map<std::string,int>::iterator it;
	it = idMap_.find(name);
	if(it == idMap_.end()) {
		idMap_[name] = idCounter_;
		return idCounter_++;
	}
	else {
		return it->second;
	}
}

const visualization_msgs::Marker& MarkerPublisher::setMarker(const PlTerm &data_term)
{
	// data_term=[Action,ID,Type,Pose,Scale,Color,Mesh,Text]
	PlTail l0(data_term);
	PlTerm e0, e1, e2;

	l0.next(e0); msg_.action = (int)e0;
	if(msg_.action == visualization_msgs::Marker::DELETEALL)
		return msg_;

	l0.next(e0); msg_.id = getID(std::string((char*)e0));
	if(msg_.action == visualization_msgs::Marker::DELETE)
		return msg_;

	l0.next(e0); msg_.type = (int)e0;

	// read pose term [frame,position,rotation]
	l0.next(e0); {
		PlTail l1(e0);
		l1.next(e1); msg_.header.frame_id = (char*)e1;
		l1.next(e1); {
			PlTail l2(e1);
			l2.next(e2); msg_.pose.position.x = (double)e2;
			l2.next(e2); msg_.pose.position.y = (double)e2;
			l2.next(e2); msg_.pose.position.z = (double)e2;
		}
		l1.next(e1); {
			PlTail l2(e1);
			l2.next(e2); msg_.pose.orientation.x = (double)e2;
			l2.next(e2); msg_.pose.orientation.y = (double)e2;
			l2.next(e2); msg_.pose.orientation.z = (double)e2;
			l2.next(e2); msg_.pose.orientation.w = (double)e2;
		}
	}

	// read scale vector
	l0.next(e0); {
		PlTail l1(e0);
		l1.next(e1); msg_.scale.x = (double)e1;
		l1.next(e1); msg_.scale.y = (double)e1;
		l1.next(e1); msg_.scale.z = (double)e1;
	}


	// read color data
	l0.next(e0); {
		PlTail l1(e0);
		l1.next(e1); msg_.color.r = (double)e1;
		l1.next(e1); msg_.color.g = (double)e1;
		l1.next(e1); msg_.color.b = (double)e1;
		l1.next(e1); msg_.color.a = (double)e1;
	}

	l0.next(e0); msg_.mesh_resource = (char*)e0;
	l0.next(e0); msg_.text = (char*)e0;

	return msg_;
}

static ros::NodeHandle node;
static MarkerPublisher pub(node);

PREDICATE(marker_array_publish, 1)
{
	visualization_msgs::MarkerArray msg;
	// populate array
	PlTail list(PL_A1);
	PlTerm member;
	while(list.next(member)) {
		msg.markers.push_back(pub.setMarker(member));
	}
	//
	pub.publish(msg);
	return true;
}
