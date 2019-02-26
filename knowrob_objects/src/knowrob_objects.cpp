#include "knowrob_objects.h"

#include <stdlib.h>
#include <iostream>
#include <condition_variable>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <rosprolog.h>
#include <knowrob_objects/ObjectState.h>
#include <knowrob_objects/ObjectStateArray.h>

class MarkDirtyObjectClient {
private:
	typedef std::map<std::string,knowrob_objects::ObjectState> ObjectStateMap;
	typedef std::pair<std::string,knowrob_objects::ObjectState> ObjectStateMapItem;
	ObjectStateMap dirty_objects[2];
	std::mutex push_lock;
	std::mutex loop_lock;
	std::condition_variable loop_cv;
	std::thread thread;
	int push_index = 0;
	bool has_dirty_objects = false;
	
public:
	MarkDirtyObjectClient() : 
	    thread(&MarkDirtyObjectClient::loop, this)
	{}
	
	void broadcast(ros::Publisher &publisher) {
		ObjectStateMap &objects = dirty_objects[push_index];
		if(objects.empty()) return;
		// ping-pong, less locking
		push_lock.lock();
		push_index = (int)!push_index;
		push_lock.unlock();
		// fill message
		knowrob_objects::ObjectStateArray msg;
		for(ObjectStateMap::iterator it = objects.begin(); it != objects.end(); ++it) {
			msg.object_states.push_back(it->second);
		}
		// send message
		publisher.publish(msg);
		objects.clear();
	}
	
	void loop() {
		ros::NodeHandle n_;
		ros::Publisher publisher_ = n_.advertise<knowrob_objects::ObjectStateArray>("/object_state", 10000);
		while(1) {
			{
				std::unique_lock<std::mutex> lk(loop_lock);
				loop_cv.wait(lk, [this]{ return has_dirty_objects; });
				has_dirty_objects = false;
			}
			broadcast(publisher_);
			std::this_thread::sleep_for(std::chrono::milliseconds(60));
		}
	}
	
	void push_begin() { push_lock.lock(); }
	void push_end()   {
		push_lock.unlock();
		{
			std::lock_guard<std::mutex> lk(loop_lock);
			has_dirty_objects = true;
		}
		loop_cv.notify_one();
	}
	
	void push(const std::string &object_id) {
		// Create a new array of term-references, all holding variables.
		PlTermv av(8);
		av[0] = object_id.c_str(); // assign first argument
		// object_information(Obj, TypeName, HasVisual, Color, Mesh, [D, W, H], Pose, StaticTransforms)
		PlQuery q("object_information",av);
		if(!q.next_solution()) {
			std::cout << "object_information returned nothing for " << object_id << std::endl;
			return;
		}
		////////////////////////
		ObjectStateMap::iterator it = dirty_objects[push_index].insert(
				ObjectStateMapItem(object_id,knowrob_objects::ObjectState())).first;
		knowrob_objects::ObjectState &obj = it->second;
		obj.object_id   = object_id.c_str();
		obj.object_type = (char*)av[1];
		obj.has_visual  = (std::string((char*)av[2]) == "true");
		obj.mesh_path   = (char*)av[4];
		std_msgs::pl_term_color(av[3], obj.color);
		geometry_msgs::pl_term_vector3(av[5], obj.size);
		geometry_msgs::pl_term_pose_stamped(av[6], obj.pose);
		{ // read static transforms
			geometry_msgs::TransformStamped staticTransform;
			PlTail staticTransforms(av[7]);
			PlTerm e;
			obj.static_transforms.clear();
			while(staticTransforms.next(e)) {
				geometry_msgs::pl_term_transform_stamped(e, staticTransform);
				obj.static_transforms.push_back(staticTransform);
			}
		}
		{ // extract frame name from pose
			PlTail pose_list(av[6]); PlTerm e;
			pose_list.next(e); // unused
			pose_list.next(e); obj.frame_name = (char*)e;
		}
	}
};

PREDICATE(mark_dirty_objects, 1) {
	static MarkDirtyObjectClient mark_dirty_client;
	PlTail tail(PL_A1); PlTerm e;
	
	mark_dirty_client.push_begin();
	while(tail.next(e)) {
		mark_dirty_client.push(std::string((char*)e));
        };
	mark_dirty_client.push_end();
	
	return TRUE;
}
