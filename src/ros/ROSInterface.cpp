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
#include "knowrob/queries/QueryError.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/modalities/KnowledgeModality.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/queries/QueryTree.h"
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <knowrob/GraphAnswerMessage.h>
#include <knowrob/GraphQueryMessage.h>
#include <knowrob/KeyValuePair.h>
#include <knowrob/askallAction.h>
#include <boost/property_tree/json_parser.hpp>
#include <utility>

using namespace knowrob;

ROSInterface::ROSInterface(const boost::property_tree::ptree& config)
        : askall_action_server_(nh_, "knowrob/askall", boost::bind(&ROSInterface::executeAskAllCB, this, _1), false),
          askone_action_server_(nh_, "knowrob/askone", boost::bind(&ROSInterface::executeAskOneCB, this, _1), false),
          askincremental_action_server_(nh_, "knowrob/askincremental", boost::bind(&ROSInterface::executeAskIncrementalCB, this, _1), false),
          tell_action_server_(nh_, "knowrob/tell", boost::bind(&ROSInterface::executeTellCB, this, _1), false),
          kb_(config)
{
    // Start all action servers
    askall_action_server_.start();
    askone_action_server_.start();
    askincremental_action_server_.start();
    tell_action_server_.start();
}

ROSInterface::~ROSInterface() = default;

FormulaPtr
ROSInterface::applyModality(const GraphQueryMessage &query,
                            FormulaPtr phi) {
    FormulaPtr mFormula = std::move(phi);

    // Add epistemic operator
    if (query.epistemicOperator == GraphQueryMessage::BELIEF) {
        if (!query.aboutAgentIRI.empty()) {
            if (query.confidence != 1.0){
                mFormula = std::make_shared<ModalFormula>(
                        BeliefModality::B(query.aboutAgentIRI, query.confidence), mFormula);
            } else {
                mFormula = std::make_shared<ModalFormula>(
                        BeliefModality::B(query.aboutAgentIRI), mFormula);
            }
        }
    } else if (query.epistemicOperator == GraphQueryMessage::KNOWLEDGE) {
        if (!query.aboutAgentIRI.empty()) {
            mFormula = std::make_shared<ModalFormula>(
                    KnowledgeModality::K(query.aboutAgentIRI),mFormula);
        }
    }
    // Add temporal operator
    if (query.temporalOperator == GraphQueryMessage::SOME_PAST) {
        if (query.minPastTimestamp != GraphQueryMessage::UNSPECIFIED_TIMESTAMP
            || query.maxPastTimestamp != GraphQueryMessage::UNSPECIFIED_TIMESTAMP) {
            if (query.minPastTimestamp == GraphQueryMessage::UNSPECIFIED_TIMESTAMP) {
                mFormula = std::make_shared<ModalFormula>(
                        PastModality::P(TimeInterval(std::nullopt,
                                                     query.maxPastTimestamp)),mFormula);
            } else if (query.maxPastTimestamp == GraphQueryMessage::UNSPECIFIED_TIMESTAMP) {
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
    } else if (query.temporalOperator == GraphQueryMessage::ALL_PAST) {
        if (query.minPastTimestamp != GraphQueryMessage::UNSPECIFIED_TIMESTAMP
            || query.maxPastTimestamp != GraphQueryMessage::UNSPECIFIED_TIMESTAMP) {
            if (query.minPastTimestamp == GraphQueryMessage::UNSPECIFIED_TIMESTAMP) {
                mFormula = std::make_shared<ModalFormula>(
                        PastModality::H(TimeInterval(std::nullopt,
                                                     query.maxPastTimestamp)),mFormula);
            } else if (query.maxPastTimestamp == GraphQueryMessage::UNSPECIFIED_TIMESTAMP) {
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

GraphAnswerMessage ROSInterface::createGraphAnswer(std::shared_ptr<const Answer> answer) {
    const SubstitutionPtr &substitution = answer->substitution();
    GraphAnswerMessage graphAnswer;
    for (const auto& pair : *substitution) {
        KeyValuePair kvpair;
        kvpair.key = pair.first.name();
        TermPtr term = pair.second;
        // Stringstream for list terms
        std::stringstream ss;

        switch(term->type()) {
            case TermType::STRING:
                kvpair.type = KeyValuePair::TYPE_STRING;
                kvpair.value_string = ((StringTerm*)term.get())->value();
                break;
            case TermType::DOUBLE:
                kvpair.type = KeyValuePair::TYPE_FLOAT;
                kvpair.value_float = ((DoubleTerm*)term.get())->value();
                break;
            case TermType::INT32:
                kvpair.type = KeyValuePair::TYPE_INT;
                kvpair.value_int = ((Integer32Term*)term.get())->value();
                break;
            case TermType::LONG:
                kvpair.type = KeyValuePair::TYPE_LONG;
                kvpair.value_long = ((LongTerm*)term.get())->value();
                break;
            case TermType::LIST:
                kvpair.type = KeyValuePair::TYPE_LIST;
                ss << ((ListTerm*)term.get());
                kvpair.value_list = ss.str();
                break;
            case TermType::VARIABLE:
                // TDDO: Can a variable be in the answer?
            case TermType::PREDICATE:
                // TDDO: Can a predicate be in the answer?
                break;
            case TermType::MODAL_OPERATOR:
                throw std::runtime_error("Modal Operator is not allowed as TermType in the answer." );
        }
        graphAnswer.substitution.push_back(kvpair);
    }
    return graphAnswer;
}

void ROSInterface::executeAskAllCB(const askallGoalConstPtr& goal)
{

    // Implement your action here
    FormulaPtr phi(QueryParser::parse(goal->query.queryString));

    FormulaPtr mPhi = applyModality(goal->query, phi);

    auto resultStream = kb_.submitQuery(mPhi, QUERY_FLAG_ALL_SOLUTIONS);
    auto resultQueue = resultStream->createQueue();

    int numSolutions_ = 0;
    askallResult result;
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
                GraphAnswerMessage answer = createGraphAnswer(nextResult);
                result.answer.push_back(answer);
                numSolutions_ += 1;
                // publish feedback
                askallFeedback feedback;
                feedback.numberOfSolutions = numSolutions_;
                askall_action_server_.publishFeedback(feedback);
            }
        }
    }

    if(numSolutions_ == 0) {
        result.status = askallResult::FALSE;
    } else {
        result.status = askallResult::TRUE;
    }
    askall_action_server_.setSucceeded(result);
}

void ROSInterface::executeAskIncrementalCB(const askincrementalGoalConstPtr& goal)
{

    // Implement your action here
    FormulaPtr phi(QueryParser::parse(goal->query.queryString));

    FormulaPtr mPhi = applyModality(goal->query, phi);

    auto resultStream = kb_.submitQuery(mPhi, QUERY_FLAG_ALL_SOLUTIONS);
    auto resultQueue = resultStream->createQueue();

    int numSolutions_ = 0;
    bool isTrue = false;
    askincrementalResult result;
    askincrementalFeedback feedback;
    while(true) {
        auto nextResult = resultQueue->pop_front();

        if(AnswerStream::isEOS(nextResult)) {
            break;
        }
        else {
            isTrue = true;
            if (nextResult->substitution()->empty()) {
                break;
            } else {
                // Publish feedback
                feedback.answer = createGraphAnswer(nextResult);
                numSolutions_ += 1;
                askincremental_action_server_.publishFeedback(feedback);
            }
        }
    }

    if(isTrue) {
        result.status = askallResult::TRUE;
    } else {
        result.status = askallResult::FALSE;
    }
    result.numberOfSolutionsFound = numSolutions_;
    askincremental_action_server_.setSucceeded(result);
}

void ROSInterface::executeAskOneCB(const askoneGoalConstPtr& goal)
{
    FormulaPtr phi(QueryParser::parse(goal->query.queryString));

    FormulaPtr mPhi = applyModality(goal->query, phi);

    auto resultStream = kb_.submitQuery(mPhi, QUERY_FLAG_ONE_SOLUTION);
    auto resultQueue = resultStream->createQueue();

    askoneResult result;
    auto nextResult = resultQueue->pop_front();

    if(AnswerStream::isEOS(nextResult)) {
        result.status = askoneResult::FALSE;
    } else {
        result.status = askoneResult::TRUE;
        if (!nextResult->substitution()->empty()) {
            GraphAnswerMessage answer = createGraphAnswer(nextResult);
            result.answer = answer;
        }
    }

    askoneFeedback  feedback;
    feedback.finished = true;
    askone_action_server_.publishFeedback(feedback);
    askone_action_server_.setSucceeded(result);
}

void ROSInterface::executeTellCB(const tellGoalConstPtr &goal) {
    FormulaPtr phi(QueryParser::parse(goal->query.queryString));

    FormulaPtr mPhi = applyModality(goal->query, phi);

    const QueryTree qt(phi);
    if(qt.numPaths()>1) {
        throw QueryError("Disjunctions are not allowed in assertions. "
                         "Appears in statement {}.", *phi);
    }
    else if(qt.numPaths()==0) {
        throw QueryError("Invalid assertion: '{}'", *phi);
    }

    std::vector<StatementData> data(qt.begin()->literals().size());
    std::vector<FramedRDFLiteral*> buf(qt.begin()->literals().size());
    uint32_t dataIndex = 0;
    for(auto &lit : qt.begin()->literals()) {
        auto modalIteration = lit->label()->modalOperators();
        buf[dataIndex] = new FramedRDFLiteral(lit, ModalityFrame(modalIteration));
        data[dataIndex++] = buf[dataIndex]->toStatementData();
    }

    tellResult result;
    tellFeedback feedback;
    if(kb_.insert(data)) {
        std::cout << "success, " << dataIndex << " statement(s) were asserted." << "\n";
        result.status = tellResult::TRUE;
        for(auto x : buf) delete x;
    }
    else {
        result.status = tellResult::TELL_FAILED;
        std::cout << "assertion failed." << "\n";
        for(auto x : buf) delete x;
    }
    feedback.finished = true;
    tell_action_server_.publishFeedback(feedback);
    tell_action_server_.setSucceeded(result);
}

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