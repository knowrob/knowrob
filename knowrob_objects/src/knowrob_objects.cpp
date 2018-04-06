#include "knowrob_objects.h"

#include <stdlib.h>
#include <iostream>
#include <condition_variable>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <knowrob_objects/DirtyObject.h>

class MarkDirtyObjectClient {
private:
	std::set<std::string> dirty_objects[2];
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
	
	void broadcast() {
		std::set<std::string> &objects = dirty_objects[push_index];
		if(objects.empty()) return;
		// ping-pong, less locking
		push_lock.lock();
		push_index = (int)!push_index;
		push_lock.unlock();
		// fill message
		knowrob_objects::DirtyObject msg;
		std::set<std::string>::iterator it;
		for(it = objects.begin(); it != objects.end(); ++it) {
			msg.request.object_ids.push_back(*it);
		}
		// send message
		if (!ros::service::call("/object_state_publisher/mark_dirty_object", msg)) {
			static bool complainedMarkDirtyNotAvailable = false;
			if(!complainedMarkDirtyNotAvailable) {
				std::cerr << "Failed to call service " <<
						"/object_state_publisher/mark_dirty_object. " <<
						"Is it running?" << std::endl;
				complainedMarkDirtyNotAvailable = true;
			}
		}
		objects.clear();
	}
	
	void loop() {
		while(1) {
			{
				std::unique_lock<std::mutex> lk(loop_lock);
				loop_cv.wait(lk, [this]{ return has_dirty_objects; });
				has_dirty_objects = false;
			}
			broadcast();
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
	
	void push(const std::string &val) {
		dirty_objects[push_index].insert(val);
	}
};

PREDICATE(mark_dirty_objects, 1) {
	static MarkDirtyObjectClient mark_dirty_client;
	PlTail tail(PL_A1); PlTerm e;
	
	mark_dirty_client.push_begin();
	while(tail.next(e)) mark_dirty_client.push(std::string((char*)e));
	mark_dirty_client.push_end();
	
	return TRUE;
}
