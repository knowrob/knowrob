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
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/modalities/KnowledgeModality.h"
#include "knowrob/terms/ListTerm.h"
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <iai_knowledge_msgs/GraphAnswer.h>
#include <iai_knowledge_msgs/GraphQuery.h>
#include <iai_knowledge_msgs/KeyValuePair.h>
#include <iai_knowledge_msgs/askallAction.h>
#include <boost/property_tree/json_parser.hpp>
#include <utility>

using namespace knowrob;

ROSInterface::ROSInterface(const boost::property_tree::ptree& config)
        : askall_action_server_(nh_, "knowrob/askall", boost::bind(&ROSInterface::executeAskAllCB, this, _1), false),
          askone_action_server_(nh_, "knowrob/askone", boost::bind(&ROSInterface::executeAskOneCB, this, _1), false),
          kb_(config)
{
    // Start all action servers
    askall_action_server_.start();
    askone_action_server_.start();
    //askiterative_action_server_.start();
}

ROSInterface::~ROSInterface() = default;

FormulaPtr
ROSInterface::applyModality(const iai_knowledge_msgs::GraphQuery &query,
                            FormulaPtr phi) {
    FormulaPtr mFormula = std::move(phi);

    // Add epistemic operator
    if (query.epistemicOperator == iai_knowledge_msgs::GraphQuery::BELIEF) {
        if (!query.aboutAgentIRI.empty()) {
            // TODO: Add confidence here if decided on API description
            mFormula = std::make_shared<ModalFormula>(
                    BeliefModality::B(query.aboutAgentIRI),mFormula);
        }
    } else if (query.epistemicOperator == iai_knowledge_msgs::GraphQuery::KNOWLEDGE) {
        if (!query.aboutAgentIRI.empty()) {
            mFormula = std::make_shared<ModalFormula>(
                    KnowledgeModality::K(query.aboutAgentIRI),mFormula);
        }
    }
    // Add temporal operator
    if (query.temporalOperator == iai_knowledge_msgs::GraphQuery::SOME_PAST) {
        if (query.minPastTimestamp != iai_knowledge_msgs::GraphQuery::UNSPECIFIED_TIMESTAMP
            || query.maxPastTimestamp != iai_knowledge_msgs::GraphQuery::UNSPECIFIED_TIMESTAMP) {
            if (query.minPastTimestamp == iai_knowledge_msgs::GraphQuery::UNSPECIFIED_TIMESTAMP) {
                mFormula = std::make_shared<ModalFormula>(
                        PastModality::P(TimeInterval(std::nullopt,
                                                     query.maxPastTimestamp)),mFormula);
            } else if (query.maxPastTimestamp == iai_knowledge_msgs::GraphQuery::UNSPECIFIED_TIMESTAMP) {
                mFormula = std::make_shared<ModalFormula>(
                        PastModality::P(TimeInterval(query.minPastTimestamp,
                                                     std::nullopt)),mFormula);
            } else {
                mFormula = std::make_shared<ModalFormula>(
                        PastModality::P(TimeInterval(query.minPastTimestamp,
                                                     query.maxPastTimestamp)),mFormula);
            }
        } else {
            mFormula = std::make_shared<ModalFormula>(
                    PastModality::P(),mFormula);
        }
    } else if (query.temporalOperator == iai_knowledge_msgs::GraphQuery::ALL_PAST) {
        if (query.minPastTimestamp != iai_knowledge_msgs::GraphQuery::UNSPECIFIED_TIMESTAMP
            || query.maxPastTimestamp != iai_knowledge_msgs::GraphQuery::UNSPECIFIED_TIMESTAMP) {
            if (query.minPastTimestamp == iai_knowledge_msgs::GraphQuery::UNSPECIFIED_TIMESTAMP) {
                mFormula = std::make_shared<ModalFormula>(
                        PastModality::H(TimeInterval(std::nullopt,
                                                     query.maxPastTimestamp)),mFormula);
            } else if (query.maxPastTimestamp == iai_knowledge_msgs::GraphQuery::UNSPECIFIED_TIMESTAMP) {
                mFormula = std::make_shared<ModalFormula>(
                        PastModality::H(TimeInterval(query.minPastTimestamp,
                                                     std::nullopt)),mFormula);
            } else {
                mFormula = std::make_shared<ModalFormula>(
                        PastModality::H(TimeInterval(query.minPastTimestamp,
                                                     query.maxPastTimestamp)),mFormula);
            }
        } else {
            mFormula = std::make_shared<ModalFormula>(
                    PastModality::H(),mFormula);
        }
    }
    return mFormula;
}

iai_knowledge_msgs::GraphAnswer ROSInterface::createGraphAnswer(std::shared_ptr<const Answer> answer) {
    const SubstitutionPtr &substitution = answer->substitution();
    iai_knowledge_msgs::GraphAnswer graphAnswer;
    for (const auto& pair : *substitution) {
        iai_knowledge_msgs::KeyValuePair kvpair;
        kvpair.key = pair.first.name();
        TermPtr term = pair.second;
        // Stringstream for list terms
        std::stringstream ss;

        switch(term->type()) {
            case TermType::STRING:
                kvpair.type = iai_knowledge_msgs::KeyValuePair::TYPE_STRING;
                kvpair.value_string = ((StringTerm*)term.get())->value();
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
                kvpair.type = iai_knowledge_msgs::KeyValuePair::TYPE_LIST;
                ss << ((ListTerm*)term.get());
                kvpair.value_list = ss.str();
                break;
            case TermType::VARIABLE:
                // TDDO: Can a variable be in the answer?
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

void ROSInterface::executeAskAllCB(const iai_knowledge_msgs::askallGoalConstPtr& goal)
{

    // Implement your action here
    FormulaPtr phi(QueryParser::parse(goal->query.queryString));

    FormulaPtr mPhi = applyModality(goal->query, phi);
    auto query = std::make_shared<ModalQuery>(mPhi, QUERY_FLAG_ALL_SOLUTIONS);

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
            if (nextResult->substitution()->empty()) {
                numSolutions_ = 1;
                break;
            } else {
                // Push one answer
                iai_knowledge_msgs::GraphAnswer answer = createGraphAnswer(nextResult);
                result.answer.push_back(answer);
                numSolutions_ += 1;
                // publish feedback
                iai_knowledge_msgs::askallFeedback feedback;
                feedback.numberOfSolutions = numSolutions_;
                askall_action_server_.publishFeedback(feedback);
            }
        }
    }

    if(numSolutions_ == 0) {
        result.status = iai_knowledge_msgs::askallResult::FALSE;
    } else {
        result.status = iai_knowledge_msgs::askallResult::TRUE;
    }
    askall_action_server_.setSucceeded(result);
}

void ROSInterface::executeAskOneCB(const iai_knowledge_msgs::askoneGoalConstPtr& goal)
{
    // Implement your action here
    FormulaPtr phi(QueryParser::parse(goal->query.queryString));

    FormulaPtr mPhi = applyModality(goal->query, phi);
    auto query = std::make_shared<ModalQuery>(mPhi, QUERY_FLAG_ONE_SOLUTION);

    auto resultStream = kb_.submitQuery(query->formula(), QUERY_FLAG_ONE_SOLUTION);
    auto resultQueue = resultStream->createQueue();

    iai_knowledge_msgs::askoneResult result;
    auto nextResult = resultQueue->pop_front();

    if(AnswerStream::isEOS(nextResult)) {
        result.status = iai_knowledge_msgs::askoneResult::FALSE;
    } else {
        result.status = iai_knowledge_msgs::askoneResult::TRUE;
        if (!nextResult->substitution()->empty()) {
            iai_knowledge_msgs::GraphAnswer answer = createGraphAnswer(nextResult);
            result.answer = answer;
        }
    }

    iai_knowledge_msgs::askoneFeedback  feedback;
    feedback.finished = true;
    askone_action_server_.publishFeedback(feedback);
    askone_action_server_.setSucceeded(result);
}

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