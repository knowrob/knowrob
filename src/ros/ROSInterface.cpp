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
#include "knowrob/queries/AnswerError.h"
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
    auto phi = QueryParser::parse(goal->query.queryString);
    auto query = std::make_shared<ModalQuery>(phi, QUERY_FLAG_ALL_SOLUTIONS);

    auto resultStream = kb_.submitQuery(query->formula(), QUERY_FLAG_ALL_SOLUTIONS);
    auto resultQueue = resultStream->createQueue();

    int numSolutions_ = 0;
    iai_knowledge_msgs::askallResult result;
    while(true) {
        auto nextResult = resultQueue->pop_front();


        if(AnswerStream::isEOS(nextResult)) {
            break;
        }
        else {
            iai_knowledge_msgs::GraphAnswer answer = createGraphAnswer(nextResult);
            result.answer.push_back(answer);

            numSolutions_ += 1;
        }
    }

    if(numSolutions_ == 0) {
        askall_action_server_.setSucceeded();
    } else {
        askall_action_server_.setSucceeded(result);
    }

}

ModalityFrame ROSInterface::genModalityFrame(const iai_knowledge_msgs::GraphQuery& query) {
    ModalityFrame mFrame;
    if (query.epistemicOperator == iai_knowledge_msgs::GraphQuery::BELIEF) {

    } else if (query.epistemicOperator == iai_knowledge_msgs::GraphQuery::KNOWLEDGE) {

    }
    return ModalityFrame();
}

iai_knowledge_msgs::GraphAnswer ROSInterface::createGraphAnswer(std::shared_ptr<const Answer> answer) {
    const SubstitutionPtr &substitution = answer->substitution();
    iai_knowledge_msgs::GraphAnswer graphAnswer;
    for (const auto& pair : *substitution) {
        iai_knowledge_msgs::KeyValuePair kvpair;
        kvpair.key = pair.first.name();
        TermPtr term = pair.second;

        switch(term->type()) {
            case TermType::STRING:
                kvpair.type = iai_knowledge_msgs::KeyValuePair::TYPE_STRING;
                kvpair.value_string = ((StringTerm*)term.get())->value();
                break;
            case TermType::VARIABLE:
                break;
            case TermType::DOUBLE:
                kvpair.type = iai_knowledge_msgs::KeyValuePair::TYPE_FLOAT;
                kvpair.value_float = ((DoubleTerm*)term.get())->value();
                break;
            case TermType::INT32:
                kvpair.type = iai_knowledge_msgs::KeyValuePair::TYPE_INT;
                kvpair.value_int = ((Integer32Term*)term.get())->value();
                break;
            case TermType::LONG:
                kvpair.type = iai_knowledge_msgs::KeyValuePair::TYPE_LONG;
                kvpair.value_long = ((LongTerm*)term.get())->value();
                break;
            case TermType::LIST:
                break;
            case TermType::PREDICATE:
                break;
            case TermType::MODAL_OPERATOR:
                throw AnswerError(*answer, *term);
                break;
        }
        graphAnswer.substitution.push_back(kvpair);
    }
    return graphAnswer;
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