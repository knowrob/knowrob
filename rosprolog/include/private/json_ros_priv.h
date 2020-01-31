#ifndef __JSON_ROS_PRIV_H__
#define __JSON_ROS_PRIV_H__

//STD
#include <string>
#include <list>
#include <map>
// ROS
#include <ros/ros.h>
// rosprolog
#include <rosprolog/JSONWrapper.h>
#include <rosprolog/MessageJSON.h>

namespace json_ros {
	/**
	 * Receives jsonified ROS messages and calls a Prolog
	 * predicate with the JSON string as an argument.
	 * 
	 * @author Daniel Beßler
	 */
	class Subscriber {
	public:
		Subscriber(const std::string &id, const std::string &topic, const term_t &callback);
		
		const std::string& id() { return id_; }
		const std::string& topic() { return topic_; }
		const term_t& callback() { return callback_; }
		
		void handle_message(const std::string &message_json);
		
	private:
		std::string id_;
		std::string topic_;
		std::string callback_str_;
		term_t callback_;
	};

	/**
	 * Send JSON-encoded ROS commands for messages, services,
	 * and actions to the *json_ros* node.
	 * 
	 * @author Daniel Beßler
	 */
	class Wrapper {
	public:
		static Wrapper& get();
		
		rosprolog::JSONWrapperResponse call(const std::string &mode, const std::string &json_data);
		
		rosprolog::JSONWrapperResponse subscribe(
			const std::string &subscriber,
			const std::string &topic,
			const std::string &json_data,
			const term_t &callback);
		
		rosprolog::JSONWrapperResponse unsubscribe(
			const std::string &subscriber,
			const std::string &topic,
			const std::string &json_data);
		
		void handle_message(const rosprolog::MessageJSONConstPtr& message);
		
	private:
		std::map< std::string, std::list<json_ros::Subscriber> > subscribers_;
		ros::ServiceClient service_;
		ros::Subscriber subscriber_;
		
		Wrapper(ros::NodeHandle &nh);
		~Wrapper();
		
		Wrapper(Wrapper const&); // Don't Implement
		void operator=(Wrapper const&);     // Don't implement
	};
};

#endif //__JSON_ROS_PRIV_H__
