#ifndef __ROSPROLOG_NODE_H__
#define __ROSPROLOG_NODE_H__

//STD
#include <string>
#include <memory>
#include <iostream>
// boost
#include <boost/shared_ptr.hpp>
// SWI Prolog
#include <SWI-Prolog.h>
// #include <SWI-cpp.h>
// rosprolog
#include <rosprolog/PrologPool.h>
#include <json_prolog_msgs/PrologQuery.h>
#include <json_prolog_msgs/PrologFinish.h>
#include <json_prolog_msgs/PrologNextSolution.h>

/**
 * ROS service interface to rosprolog
 *
 * @author Daniel Beßler
 */
class PrologNode {
public:
	PrologNode(ros::NodeHandle *node);
	
	bool query(
		json_prolog_msgs::PrologQuery::Request &req,
		json_prolog_msgs::PrologQuery::Response &res);
	
	bool finish(
		json_prolog_msgs::PrologFinish::Request &req,
		json_prolog_msgs::PrologFinish::Response &res);
	
	bool next_solution(
		json_prolog_msgs::PrologNextSolution::Request &req,
		json_prolog_msgs::PrologNextSolution::Response &res);
	
	bool is_initialized() { return is_initialized_; }
	
private:
	std::map< std::string, boost::shared_ptr<PrologEngine> > claimed_engines_;
	PrologPool thread_pool_;
	
	bool is_initialized_;
	
	bool exists(const std::string &id);
	
	void finish(const std::string &id);
	void finish();
	
	bool has_more_solutions(const std::string &id);
	
	static int call1(const std::string &predicate, const std::string &arg1);
	
	static int ensure_loaded(const char *ros_pkg);
	
	static int num_pl_threads(ros::NodeHandle *node);
};

#endif //__ROSPROLOG_NODE_H__
