
// STD
#include <set>
#include <map>
#include <string>
#include <mutex>
// boost
#include <boost/shared_ptr.hpp>
// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// SWI prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

typedef boost::shared_ptr<visualization_msgs::Marker> MarkerPtr;
typedef std::map<std::string,MarkerPtr> MarkerMap;
typedef std::set<MarkerPtr> MarkerQueue;

class MarkerPublisher {
public:
	static MarkerPtr get(const std::string &name) {
		return get_publisher().get_or_create(name);
	}
	
	static bool erase(const std::string &name) {
		get_publisher().locked_erase(name);
		return TRUE;
	}
	
	static bool queue(const std::string &name) {
		MarkerPtr m = get_publisher().get_or_create(name);
		get_publisher().do_queue(m);
		return TRUE;
	}
	
	static bool publish() {
		get_publisher().do_publish();
		return TRUE;
	}
	
private:
	static MarkerPublisher& get_publisher() {
		static MarkerPublisher pub;
		return pub;
	}
	
	MarkerPublisher() : 
		n_(),
		pub_(n_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",80)),
		id_(0)
	{}
	
	MarkerPtr get_or_create(const std::string &name) {
		MarkerMap::iterator needle = marker_map_.find(name);
		if(needle != marker_map_.end()) {
			return needle->second;
		} else {
			return create(name);
		}
	}
	
	MarkerPtr create(const std::string &name) {
		MarkerPtr m(new visualization_msgs::Marker());
		m->header.frame_id = "/map"; // FIXME: set in prolog
		m->header.stamp = ros::Time::now();
		m->ns = name;
		m->id = id_++;
		m->action = visualization_msgs::Marker::ADD;
		{
			std::unique_lock<std::mutex> lk(marker_m_);
			marker_map_[name] = m;
		}
		do_queue(m);
		return m;
	}
	
	void locked_erase(const std::string &name) {
		std::unique_lock<std::mutex> lk(marker_m_);
		do_erase(name);
	}
	
	void clear() {
		std::unique_lock<std::mutex> lk(marker_m_);
		while(marker_map_.begin() != marker_map_.end())
			do_erase(marker_map_.begin()->first);
	}
	
	void do_erase(const std::string &name) {
		MarkerMap::iterator needle = marker_map_.find(name);
		if(needle != marker_map_.end()) {
			boost::shared_ptr<visualization_msgs::Marker> &m = needle->second;
			m->action = visualization_msgs::Marker::DELETE;
			do_queue(m);
			marker_map_.erase(needle);
		}
	}
	
	void do_publish() {
		visualization_msgs::MarkerArray arr;
		{
			std::unique_lock<std::mutex> lk(marker_m_);
			for(MarkerQueue::iterator it=marker_queue_.begin(); it!=marker_queue_.end(); ++it)
			{
				arr.markers.push_back(*(*it));
			}
			marker_queue_.clear();
		}
		if(arr.markers.size() > 0) {
			pub_.publish(arr);
		}
	}
	
	void do_queue(boost::shared_ptr<visualization_msgs::Marker> &m) {
		std::unique_lock<std::mutex> lk(marker_m_);
		marker_queue_.insert(m);
	}

private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	MarkerMap marker_map_;
	MarkerQueue marker_queue_;
	std::mutex marker_m_;
	unsigned int id_;
};

PREDICATE(marker_publish, 0) {
	return MarkerPublisher::publish();
};

PREDICATE(marker_queue, 1) {
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(marker_erase, 1) {
	return MarkerPublisher::erase((char*)PL_A1);
};

/************************************
 * Getter/Setter for marker messages
 ************************************/

PREDICATE(get_marker_id, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PL_A2 = m->id;
	return TRUE;
};

PREDICATE(get_marker_ns, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PL_A2 = m->ns.c_str();
	return TRUE;
};
  
PREDICATE(get_marker_duration, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PL_A2 = m->lifetime.toSec();
	return TRUE;
};
PREDICATE(set_marker_duration, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	m->lifetime = ros::Duration(PL_A2);
// 	for(MarkerObject child : children) child.setLifetime(value);
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_timestamp, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PL_A2 = m->header.stamp.toSec();
	return TRUE;
};
PREDICATE(set_marker_timestamp, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	double posix_ts = PL_A2;
	m->header.stamp.fromSec(posix_ts);
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_type, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PL_A2 = m->type;
	return TRUE;
};
PREDICATE(set_marker_type, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	m->type = PL_A2;
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_mesh, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PL_A2 = m->mesh_resource.c_str();
	return TRUE;
};
PREDICATE(set_marker_mesh, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	m->mesh_resource = std::string((char*)PL_A2);
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_text, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PL_A2 = m->text.c_str();
	return TRUE;
};
PREDICATE(set_marker_text, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	m->text = std::string((char*)PL_A2);
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_scale, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail l(PL_A2);
	l.append(m->scale.x);
	l.append(m->scale.y);
	l.append(m->scale.z);
	l.close();
	return TRUE;
};
PREDICATE(set_marker_scale, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail list(PL_A2); PlTerm value;
	list.next(value); m->scale.x = value;
	list.next(value); m->scale.y = value;
	list.next(value); m->scale.z = value;
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_color, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail l(PL_A2);
	l.append(m->color.r);
	l.append(m->color.g);
	l.append(m->color.b);
	l.append(m->color.a);
	l.close();
	return TRUE;
};
PREDICATE(set_marker_color, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail list(PL_A2); PlTerm value;
	list.next(value); m->color.r = (double)value;
	list.next(value); m->color.g = (double)value;
	list.next(value); m->color.b = (double)value;
	list.next(value); m->color.a = (double)value;
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_colors, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail colors_l(PL_A2);
	for(const std_msgs::ColorRGBA &x : m->colors) {
		PlTerm color_term;
		PlTail color_l(color_term);
		color_l.append(x.r);
		color_l.append(x.g);
		color_l.append(x.b);
		color_l.append(x.a);
		color_l.close();
		colors_l.append(color_term);
	}
	colors_l.close();
	return TRUE;
};
PREDICATE(set_marker_colors, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail colors_l(PL_A2); PlTerm i;
	//
	m->colors.clear();
	while(colors_l.next(i)) {
		PlTail color_l(i); PlTerm j;
		std_msgs::ColorRGBA color;
		color_l.next(j); color.r = (double)j;
		color_l.next(j); color.g = (double)j;
		color_l.next(j); color.b = (double)j;
		color_l.next(j); color.a = (double)j;
		m->colors.push_back(color); // TODO: avoid copy
	}
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_alpha, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PL_A2 = m->color.a;
	return TRUE;
};
PREDICATE(set_marker_alpha, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	m->color.a = (double)PL_A2;
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_translation, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail l(PL_A2);
	l.append(m->pose.position.x);
	l.append(m->pose.position.y);
	l.append(m->pose.position.z);
	l.close();
	return TRUE;
};
PREDICATE(set_marker_translation, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail list(PL_A2); PlTerm value;
	list.next(value); m->pose.position.x = value;
	list.next(value); m->pose.position.y = value;
	list.next(value); m->pose.position.z = value;
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_orientation, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail l(PL_A2);
	l.append(m->pose.orientation.x);
	l.append(m->pose.orientation.y);
	l.append(m->pose.orientation.z);
	l.append(m->pose.orientation.w);
	l.close();
	return TRUE;
};
PREDICATE(set_marker_orientation, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail list(PL_A2); PlTerm value;
	list.next(value); m->pose.orientation.x = value;
	list.next(value); m->pose.orientation.y = value;
	list.next(value); m->pose.orientation.z = value;
	list.next(value); m->pose.orientation.w = value;
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_pose, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	//
	PlTerm frame_id(m->header.frame_id.c_str());
	PlTerm marker_id;
	PlTerm position; {
		PlTail pos_l(position);
		pos_l.append(m->pose.position.x);
		pos_l.append(m->pose.position.y);
		pos_l.append(m->pose.position.z);
		pos_l.close();
	}
	PlTerm orientation; {
		PlTail rot_l(orientation);
		rot_l.append(m->pose.orientation.x);
		rot_l.append(m->pose.orientation.y);
		rot_l.append(m->pose.orientation.z);
		rot_l.append(m->pose.orientation.w);
		rot_l.close();
	}
	//
	PlTail pose_l(PL_A2);
	pose_l.append(frame_id);
	pose_l.append(marker_id);
	pose_l.append(position);
	pose_l.append(orientation);
	pose_l.close();
	return TRUE;
};
PREDICATE(set_marker_pose, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail pose_l(PL_A2); PlTerm i;
	pose_l.next(i); m->header.frame_id = (char*)i;
	pose_l.next(i);
	pose_l.next(i); {
		PlTail pos_l(i); PlTerm j;
		pos_l.next(j); m->pose.position.x = j;
		pos_l.next(j); m->pose.position.y = j;
		pos_l.next(j); m->pose.position.z = j;
	}
	pose_l.next(i); {
		PlTail rot_l(i); PlTerm j;
		rot_l.next(j); m->pose.orientation.x = j;
		rot_l.next(j); m->pose.orientation.y = j;
		rot_l.next(j); m->pose.orientation.z = j;
		rot_l.next(j); m->pose.orientation.w = j;
	}
	return MarkerPublisher::queue((char*)PL_A1);
};

PREDICATE(get_marker_points, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail points_l(PL_A2);
	for(const geometry_msgs::Point &p : m->points) {
		PlTerm point_term;
		PlTail point_l(point_term);
		point_l.append(p.x);
		point_l.append(p.y);
		point_l.append(p.z);
		point_l.close();
		points_l.append(point_term);
	}
	points_l.close();
	return TRUE;
};
PREDICATE(set_marker_points, 2) {
	MarkerPtr m = MarkerPublisher::get((char*)PL_A1);
	PlTail points_l(PL_A2); PlTerm i;
	//
	m->points.clear();
	while(points_l.next(i)) {
		PlTail point_l(i); PlTerm j;
		geometry_msgs::Point p;
		point_l.next(j); p.x = j;
		point_l.next(j); p.y = j;
		point_l.next(j); p.z = j;
		m->points.push_back(p); // TODO: avoid copy
	}
	return MarkerPublisher::queue((char*)PL_A1);
};
