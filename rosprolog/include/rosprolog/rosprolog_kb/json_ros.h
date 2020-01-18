#ifndef __JSON_ROS_H__
#define __JSON_ROS_H__

//STD
#include <string>
#include <list>
#include <map>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-Prolog.h>
// rosprolog
#include <rosprolog/JSONWrapper.h>

namespace json_ros {
	/**
	 * Subscribe to a ROS topic by sending a JSON-encoded
	 * command to the *json_ros* node.
	 */
	static rosprolog::JSONWrapperResponse subscribe(
			const std::string &subscriber,
			const std::string &topic,
			const std::string &json_data,
			const term_t &goal);
	
	/**
	 * Unsubscribe from a ROS topic by sending a JSON-encoded
	 * command to the *json_ros* node.
	 */
	static rosprolog::JSONWrapperResponse unsubscribe(
			const std::string &subscriber,
			const std::string &topic,
			const std::string &json_data);
	
	/**
	 * Publish on a ROS topic by sending a JSON-encoded
	 * command to the *json_ros* node.
	 */
	static rosprolog::JSONWrapperResponse publish(
			const std::string &topic,
			const std::string &json_data);
	
	/**
	 * Invoke a ROS service by sending a JSON-encoded
	 * command to the *json_ros* node.
	 */
	static rosprolog::JSONWrapperResponse call_service(
			const std::string &service_path,
			const std::string &json_data);
	
	/**
	 * Invoke a ROS action by sending a JSON-encoded
	 * command to the *json_ros* node.
	 */
// 	static rosprolog::JSONWrapperResponse call_action(
// 			const std::string &action_path,
// 			const std::string &json_data,
// 			const term_t &update_callback);
	
	class Exception : public std::exception
	{
		std::string _msg;
	public:
		Exception(const std::string& msg) : _msg(msg){}
		
		virtual const char* what() const noexcept override
		{
			return _msg.c_str();
		}
	}; 
};

#endif //__JSON_WRAPPE_CLIENT_H__
