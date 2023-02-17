/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <termios.h>
// STD
#include <exception>
#include <iostream>
// BOOST
#include <boost/program_options/variables_map.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
// KnowRob
#include <knowrob/knowrob.h>
#include <knowrob/Logger.h>
#include <knowrob/HybridQA.h>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <json_prolog_msgs/PrologQuery.h>
#include <json_prolog_msgs/PrologFinish.h>
#include <json_prolog_msgs/PrologNextSolution.h>
// ResultHandler
#include "IterativeQAResultHandler.cpp"

#define PARAM_INITIAL_PACKAGE "initial_package"
#define PARAM_INITIAL_GOAL "initial_goal"
#define PARAM_NUM_ROS_THREADS "num_ros_threads"

#define NUM_ROS_THREADS_DEFAULT 2

using namespace knowrob;
namespace po = boost::program_options;

std::map< std::string, std::shared_ptr<IterativeQAResultHandler> > resultHandlers_;
boost::property_tree::ptree config;
bool is_initialized_ = false;
HybridQA* hybridQA_;
int numSolutions_;

void runQuery(const std::string &queryString, IterativeQAResultHandler &handler) {
    try {
        // parse query
        auto query = hybridQA_->parseQuery(queryString);
        // evaluate query in hybrid QA system
        numSolutions_ = 0;
        hybridQA_->runQuery(query, handler);
        if(handler.getNumSolutions() == 0) {
            handler.pushSolution("False.");
        }
    }
    catch (std::exception& e) {
        KB_ERROR("ERROR: {}.", e.what());
        handler.setError(e.what());
    }
}

bool exists(const std::string &id)
{
    return resultHandlers_.find(id) != resultHandlers_.end();
}

bool has_more_solutions(const std::string &id)
{
    return resultHandlers_.find(id)->second->has_more_solutions();
}

bool is_initialized() {
    return is_initialized;
}

void finish(const std::string &id)
{
    auto it = resultHandlers_.find(id);
    if(it != resultHandlers_.end()) {
        it->second->finish();
        resultHandlers_.erase(it);
    }
}

void finish()
{
    auto it = resultHandlers_.begin();
    while (it != resultHandlers_.end()) {
        finish(it->first);
        it = resultHandlers_.begin();
    }
}

bool query(json_prolog_msgs::PrologQuery::Request &req,
           json_prolog_msgs::PrologQuery::Response &res)
{
    if (exists(req.id)) {
        std::stringstream ss;
        ss << "Another query is already being processed with id " << req.id << '.';
        res.ok = false;
        res.message = ss.str();
    } else {
        resultHandlers_[req.id] = std::make_shared<IterativeQAResultHandler>(config, req.query, req.mode);
        runQuery(req.query, *resultHandlers_[req.id]);
        res.ok = true;
        res.message = "";
    }
    return true;
}

bool next_solution(json_prolog_msgs::PrologNextSolution::Request &req,
                   json_prolog_msgs::PrologNextSolution::Response &res)
{
    if(!exists(req.id)) {
        res.status = json_prolog_msgs::PrologNextSolution::Response::WRONG_ID;
        res.solution = "";
    }
    else {
        if (!has_more_solutions(req.id)){
            std::shared_ptr<IterativeQAResultHandler> x = resultHandlers_.find(req.id)->second;
            if(x->has_error()) {
                res.status = json_prolog_msgs::PrologNextSolution::Response::QUERY_FAILED;
                res.solution = x->error();
                ROS_WARN("%s.", res.solution.c_str());
            }
            else {
                res.status = json_prolog_msgs::PrologNextSolution::Response::NO_SOLUTION;
                res.solution = "";
            }
        }
        else {
            res.status = json_prolog_msgs::PrologNextSolution::Response::OK;
            res.solution  =
                    resultHandlers_.find(req.id)->second->next_solution();
        }
    }
    return true;
}

bool finish(json_prolog_msgs::PrologFinish::Request &req,
            json_prolog_msgs::PrologFinish::Response &res)
{
    if (req.id == "*"){
        // finish all queries
        finish();
    } else {
        finish(req.id);
    }
    return true;
}

// TODO: Decide if loading specific init.pls is still necessary?
//int ensure_loaded(HybridQA hybridQA_, const char *ros_pkg)
//{
//    std::stringstream ss;
//    ss << ros::package::getPath(ros_pkg) << "/src/__init__.pl";
//    if(!hybridQA_.callPrologDirect("ensure_loaded(" + ss.str() + ").")) {
//        ROS_ERROR("Failed to load __init__.pl of %s.", ros_pkg);
//        return FALSE;
//    }
//    return TRUE;
//}

int initializeRos(ros::NodeHandle node) {
    // TODO: Decide if loading specific init.pls is still necessary?
//    if(!ensure_loaded(hybridQA_, "knowrob")) {
//        return FALSE;
//    }
    std::string param;
    // register initial packages
    if (node.getParam(PARAM_INITIAL_PACKAGE, param)) {
        if(!hybridQA_->callPrologDirect("register_ros_package(" + param + ")")) {
            ROS_ERROR("Failed to load initial_package %s.", param.c_str());
            return FALSE;
        }
    }
    else if(!hybridQA_->callPrologDirect("register_ros_package(knowrob).")) {
        ROS_ERROR("Failed to load knowrob.");
        return FALSE;
    }
    // execute initial goal
    if (node.getParam(PARAM_INITIAL_GOAL, param)) {
        IterativeQAResultHandler qaHandler_(config, param, false);
        runQuery(param, qaHandler_);
        if(qaHandler_.has_error()) {
            ROS_WARN("initial goal failed: %s", qaHandler_.error().c_str());
        }
        qaHandler_.finish();
    }
    is_initialized_ = true;
    return is_initialized_;
}

sig_atomic_t volatile g_request_shutdown = 0;
void sigint_handler(int sig)
{
    g_request_shutdown=1;
}

int run(int argc, char **argv) {
    // Check for settings file
    std::string config_path = "default.json";
    if(std::getenv("KNOWROB_SETTINGS")) {
        config_path = std::getenv("KNOWROB_SETTINGS");
    }

    // read settings
    boost::property_tree::read_json(config_path, config);

    // init hybrid qa
    hybridQA_ = new HybridQA(config);

    // configure logging
    auto &log_config = config.get_child("logging");
    if(!log_config.empty()) {
        Logger::loadConfiguration(log_config);
    }
    // overwrite console logger level (default: prevent messages being printed, only print errors)
    Logger::setSinkLevel(Logger::Console, spdlog::level::debug);

    ros::init(argc, argv, "rosprolog", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    // rosprolog can serve requests in parallel
    int num_ros_threads=0;
    if(!n.getParam(PARAM_NUM_ROS_THREADS, num_ros_threads)) {
        num_ros_threads = NUM_ROS_THREADS_DEFAULT;
    }
    ros::AsyncSpinner spinner(num_ros_threads);
    // loop at 10Hz
    ros::Rate loop_rate(10.0); //Hz
    spinner.start();
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, sigint_handler);
    //
    // initializeRos(n);
    is_initialized_ = true;
    if(is_initialized()) {
        ros::ServiceServer service_query = n.advertiseService(
                "/rosprolog/query", query);
        ros::ServiceServer service_next_solution = n.advertiseService(
                "/rosprolog/next_solution", next_solution);
        ros::ServiceServer service_finish = n.advertiseService(
                "/rosprolog/finish", finish);

        ROS_INFO("rosprolog service is running.");
        while (ros::ok() && !g_request_shutdown) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        ROS_INFO("rosprolog service is exiting.");
        finish();
        ros::shutdown();

        return EXIT_SUCCESS;
    }
    else {
        ROS_ERROR("rosprolog service failed to initialize.");
        return EXIT_FAILURE;
    }
}

int main(int argc, char **argv) {
	InitKnowledgeBase(argc, argv);
	try {
		return run(argc,argv);
	}
	catch(std::exception& e) {
		KB_ERROR("an exception occurred: {}.", e.what());
		return EXIT_FAILURE;
	}
}
