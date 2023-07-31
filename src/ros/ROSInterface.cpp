/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/ros/ROSInterface.h"
// KnowRob
#include "knowrob/knowrob.h"
#include "knowrob/Logger.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/queries/QueryParser.h"
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <iai_knowledge_msgs/GraphAnswer.h>
#include <iai_knowledge_msgs/GraphQuery.h>
#include <iai_knowledge_msgs/KeyValuePair.h>
#include <iai_knowledge_msgs/askallAction.h>
#include <boost/property_tree/json_parser.hpp>

using namespace knowrob;

ROSInterface::ROSInterface(boost::property_tree::ptree config)
        : askall_action_server_(nh_, "knowrob/askall", boost::bind(&ROSInterface::executeAskAllCB, this, _1), false),
          kb_(config)
{
    // Start all action servers
    askall_action_server_.start();
    //askone_action_server_.start();
    //askiterative_action_server_.start();
}

ROSInterface::~ROSInterface() = default;


void ROSInterface::executeAskAllCB(const iai_knowledge_msgs::askallGoalConstPtr& goal)
{
    // Implement your action here
    ModalityFrame mfram = genModalityFrame(goal->query);

    // By default, saying the action was successful
    askall_action_server_.setSucceeded();
}

ModalityFrame ROSInterface::genModalityFrame(iai_knowledge_msgs::askallGoal_<std::allocator<void>>::_query_type query) {

    return ModalityFrame();
}


//void ROSInterface::executeAskOneCB(const iai_knowledge_msgs::askoneGoalConstPtr& goal)
//{
//    // Implement your action here
//
//    // By default, saying the action was successful
//    askone_action_server_.setSucceeded();
//}

//void ROSInterface::executeAskIterativeCB(const iai_knowledge_msgs::askiterativeGoalConstPtr& goal)
//{
//    // Implement your action here
//
//    // By default, saying the action was successful
//    askiterative_action_server_.setSucceeded();
//}

boost::property_tree::ptree loadSetting() {
    // Check for settings file
    std::string config_path = "default.json";
    if(std::getenv("KNOWROB_SETTINGS")) {
        config_path = std::getenv("KNOWROB_SETTINGS");
    }

    // read the settings
    boost::property_tree::ptree config;
    boost::property_tree::read_json(
            config_path,
            config);

    return config;

}

int main(int argc, char **argv) {
	InitKnowledgeBase(argc, argv);

    // Load settings files
    loadSetting();
	try {
        ros::init(argc, argv, "knowrob_node");
        ROSInterface ros_interface(loadSetting());
        ros::spin();
	}
	catch(std::exception& e) {
		KB_ERROR("an exception occurred: {}.", e.what());
		return EXIT_FAILURE;
	}
}