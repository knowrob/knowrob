#include "rosprolog/rosprolog_kb/rosprolog_kb.h"
#include "rosprolog/rosprolog_kb/json_ros.h"
#include "private/json_ros_priv.h"

#include <stdlib.h>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>

/*****************************
 ******** JSON-Subscriber
 ****************************/

namespace json_ros {
	Subscriber::Subscriber(
			const std::string &id,
			const std::string &topic,
			const term_t &callback) :
		id_(id),
		topic_(topic),
		callback_(callback)
	{
		char *term_chars=NULL;
		if(PL_get_chars(callback, &term_chars, CVT_WRITE|BUF_RING)) {
			callback_str_=std::string(term_chars);
		} else {
			throw json_ros::Exception("failed to create subscriber");
		}
	}

	void Subscriber::handle_message(const std::string &message_json)
	{
		std::stringstream ss;
		ss << "ros_callback(" << callback_str_ << ",'" << message_json << "')";
		//
		boost::shared_ptr<PrologEngine> engine = rosprolog_kb::thread_pool().claim();
		engine->one_solution(ss.str());
		rosprolog_kb::thread_pool().release(engine);
	}
};

/*****************************
 ******** JSON_ROS
 ****************************/
    
namespace json_ros {
	Wrapper::Wrapper(ros::NodeHandle &nh):
		service_(nh.serviceClient<rosprolog::JSONWrapper>("/json_wrapper")),
		subscriber_(nh.subscribe("/json/message", 1, &Wrapper::handle_message, this))
	{}
	
	Wrapper::~Wrapper()
	{}
	
	Wrapper& Wrapper::get()
	{
		static Wrapper the_wrapper(rosprolog_kb::node());
		return the_wrapper;
	}
	    
	rosprolog::JSONWrapperResponse Wrapper::call(const std::string &mode, const std::string &json_data)
	{
		rosprolog::JSONWrapper msg;
		msg.request.mode = mode;
		msg.request.json_data = json_data;
		if(service_.call(msg)) {
			return msg.response;
		} else {
			throw json_ros::Exception("failed to publish json message");
		}
	}
	    
	rosprolog::JSONWrapperResponse Wrapper::subscribe(
			const std::string &subscriber,
			const std::string &topic,
			const std::string &json_data,
			const term_t &callback)
	{
		subscribers_[topic].push_back(json_ros::Subscriber(
			subscriber, topic, callback));
		if(subscribers_[topic].size()==1) {
			return call("subscribe",json_data);
		} else {
			return rosprolog::JSONWrapperResponse();
		}
	}
	    
	rosprolog::JSONWrapperResponse Wrapper::unsubscribe(
			const std::string &subscriber,
			const std::string &topic,
			const std::string &json_data)
	{
		std::list< json_ros::Subscriber > &l = subscribers_[topic];
		for(std::list< json_ros::Subscriber >::iterator it=l.begin(); it!=l.end(); ++it) {
			if(it->id().compare(subscriber)==0) {
				l.erase(it);
				break;
			}
		}
		if(l.empty()) {
			return call("unsubscribe",json_data);
		} else {
			return rosprolog::JSONWrapperResponse();
		}
	}

	void Wrapper::handle_message(const rosprolog::MessageJSONConstPtr& message)
	{
		std::list< json_ros::Subscriber > &l = subscribers_[message->topic_name];
		for(std::list< json_ros::Subscriber >::iterator it=l.begin(); it!=l.end(); ++it) {
			it->handle_message(message->json_data);
		}
	}
};

/*****************************
 ******** ....
 ****************************/

namespace json_ros {
	rosprolog::JSONWrapperResponse subscribe(
			const std::string &subscriber,
			const std::string &topic,
			const std::string &json_data,
			const term_t &goal)
	{
		return json_ros::Wrapper::get().subscribe(subscriber,topic,json_data,goal);
	}
	    
	rosprolog::JSONWrapperResponse unsubscribe(
			const std::string &subscriber,
			const std::string &topic,
			const std::string &json_data)
	{
		return json_ros::Wrapper::get().unsubscribe(subscriber,topic,json_data);
	}
	    
	rosprolog::JSONWrapperResponse publish(
			const std::string &topic,
			const std::string &json_data)
	{
		return json_ros::Wrapper::get().call("publish",json_data);
	}

	rosprolog::JSONWrapperResponse call_service(
			const std::string &service_path,
			const std::string &json_data)
	{
		return json_ros::Wrapper::get().call("service",json_data);
	}

// 	rosprolog::JSONWrapperResponse call_action(
// 			const std::string &action_path,
// 			const std::string &json_data,
// 			const term_t &update_callback)
// 	{
// 		return json_ros::Wrapper::get().call("action",json_data);
// 	}
}

/*****************************
 ******** PL predicates
 ****************************/

PREDICATE(json_ros_subscribe, 4) {
	std::string subscriber = std::string((char*)PL_A1);
	std::string topic = std::string((char*)PL_A2);
	std::string json_data = std::string((char*)PL_A3);
	term_t callback = (term_t)PL_A4;
	try {
		json_ros::subscribe(subscriber,topic,json_data,callback);
		return TRUE;
	}
	catch (...) {
		ROS_ERROR("Failed to invoke json_wrapper, is it running?");
		return FALSE;
	}
}

PREDICATE(json_ros_unsubscribe, 3) {
	std::string subscriber = std::string((char*)PL_A1);
	std::string topic = std::string((char*)PL_A2);
	std::string json_data = std::string((char*)PL_A3);
	try {
		json_ros::unsubscribe(subscriber,topic,json_data);
		return TRUE;
	}
	catch (...) {
		ROS_ERROR("Failed to invoke json_wrapper, is it running?");
		return FALSE;
	}
}

PREDICATE(json_ros_publish, 2) {
	std::string topic = std::string((char*)PL_A1);
	std::string json_data = std::string((char*)PL_A2);
	try {
		rosprolog::JSONWrapperResponse response = json_ros::publish(topic,json_data);
		return TRUE;
	}
	catch (...) {
		ROS_ERROR("Failed to invoke json_wrapper, is it running?");
		return FALSE;
	}
}

PREDICATE(json_ros_service_call, 3) {
	std::string service_path = std::string((char*)PL_A1);
	std::string json_data = std::string((char*)PL_A2);
	try {
		rosprolog::JSONWrapperResponse response = json_ros::call_service(service_path,json_data);
		PL_A3 = response.json_data.c_str();
		return TRUE;
	}
	catch (...) {
		ROS_ERROR("Failed to invoke json_wrapper, is it running?");
		return FALSE;
	}
}
