#include <ros/ros.h>

#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <stdio.h>
#include <dlfcn.h>
#include <iostream>
#include <string>
#include <memory>

#include <stdlib.h>
#include <condition_variable>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <rosprolog/rosprolog_kb/rosprolog_kb.h>
#include <knowrob/ObjectState.h>
#include <knowrob/ObjectStateArray.h>

class MarkDirtyObjectClient {
private:
	typedef std::map<std::string,knowrob::ObjectState> ObjectStateMap;
	typedef std::pair<std::string,knowrob::ObjectState> ObjectStateMapItem;
	ObjectStateMap dirty_objects[2];
	std::set<std::string> deleted_objects[2];
	std::mutex push_lock;
	std::mutex loop_lock_;
	std::condition_variable loop_cv_;
	std::thread thread_;
	int push_index = 0;
	bool has_dirty_objects = false;
	bool is_terminated_ = false;
	
public:
	MarkDirtyObjectClient() : 
	    thread_(&MarkDirtyObjectClient::loop, this)
	{}
	
	~MarkDirtyObjectClient()
	{
		is_terminated_ = true;
		loop_cv_.notify_one();
		thread_.join();
	}
	
	void broadcast(ros::Publisher &publisher) {
		ObjectStateMap &objects = dirty_objects[push_index];
		std::set<std::string> &deleted = deleted_objects[push_index];
		if(objects.empty() && deleted.empty()) return;
		// ping-pong, less locking
		push_lock.lock();
		push_index = (int)!push_index;
		push_lock.unlock();
		if(!objects.empty()) {
			knowrob::ObjectStateArray msg;
			msg.action = knowrob::ObjectStateArray::ADD;
			for(ObjectStateMap::iterator it = objects.begin(); it != objects.end(); ++it) {
				msg.object_states.push_back(it->second);
			}
			publisher.publish(msg);
			objects.clear();
		}
		if(!deleted.empty()) {
			knowrob::ObjectStateArray msg;
			msg.action = knowrob::ObjectStateArray::DELETE;
			for(std::set<std::string>::iterator it = deleted.begin(); it != deleted.end(); ++it) {
				knowrob::ObjectState os_msg;
				os_msg.object_id = (*it).c_str();
				msg.object_states.push_back(os_msg);
			}
			publisher.publish(msg);
			deleted.clear();
		}
	}
	
	void loop() {
		ros::NodeHandle n_;
		ros::Publisher publisher_ = n_.advertise<knowrob::ObjectStateArray>("/object_state", 10000);
		while(ros::ok()) {
			{
				std::unique_lock<std::mutex> lk(loop_lock_);
				if(is_terminated_) break;
				loop_cv_.wait(lk, [this]{ return has_dirty_objects || is_terminated_; });
				has_dirty_objects = false;
				if(is_terminated_) break;
			}
			broadcast(publisher_);
			std::this_thread::sleep_for(std::chrono::milliseconds(60));
		}
	}
	
	void push_begin() { push_lock.lock(); }
	void push_end()   {
		push_lock.unlock();
		{
			std::lock_guard<std::mutex> lk(loop_lock_);
			has_dirty_objects = true;
		}
		loop_cv_.notify_one();
	}
	
	void push(const PlTerm &data) {
		PlTail l_data(data); PlTerm e;
		l_data.next(e); // object_id
		std::string object_id((char*)e);
		////////////////////////
		ObjectStateMap::iterator it = dirty_objects[push_index].insert(
				ObjectStateMapItem(object_id,knowrob::ObjectState())).first;
		knowrob::ObjectState &obj = it->second;
		obj.object_id   = object_id.c_str();
		l_data.next(e); obj.frame_name  = (char*)e;
		l_data.next(e); obj.object_type = (char*)e;
		l_data.next(e); obj.shape       = (int)e;
		l_data.next(e); obj.mesh_path   = (char*)e;
		l_data.next(e); rosprolog_kb::term_to_color(e, obj.color);
		l_data.next(e); rosprolog_kb::term_to_vector3(e, obj.size);
		l_data.next(e); rosprolog_kb::term_to_pose_stamped(e, obj.pose);
		l_data.next(e);
		{ // read static transforms
			geometry_msgs::TransformStamped staticTransform;
			PlTail staticTransforms(e);
			PlTerm e;
			obj.static_transforms.clear();
			while(staticTransforms.next(e)) {
				rosprolog_kb::term_to_transform_stamped(e, staticTransform);
				obj.static_transforms.push_back(staticTransform);
			}
		}
	}
	
	void push_deleted(const PlTerm &data) {
		std::string object_id((char*)data);
		deleted_objects[push_index].insert(object_id);
	}
};

PREDICATE(object_state_add_cpp, 1) {
	static MarkDirtyObjectClient mark_dirty_client;
	PlTail tail(PL_A1); PlTerm e;

	mark_dirty_client.push_begin();
	while(tail.next(e)) {
		mark_dirty_client.push(e);
        };
	mark_dirty_client.push_end();

	return TRUE;
}

PREDICATE(object_state_remove_cpp, 1) {
	static MarkDirtyObjectClient mark_dirty_client;
	PlTail tail(PL_A1); PlTerm e;

	mark_dirty_client.push_begin();
	while(tail.next(e)) {
		mark_dirty_client.push_deleted(e);
        };
	mark_dirty_client.push_end();

	return TRUE;
}
