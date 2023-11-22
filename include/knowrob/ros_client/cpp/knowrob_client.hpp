/**
 * @file knowrob_client.hpp
 * @author Marco Masannek
 *
 * @brief Cpp client class for the knowrob action interfaces
 *
 * @version 0.1
 * @date 2023-11-21
 *
 * @copyright Copyright (c) 2023 Marco Masannek
 *
 */

#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <knowrob/GraphAnswerMessage.h>
#include <knowrob/GraphQueryMessage.h>

#include <knowrob/askoneAction.h>
#include <knowrob/askincrementalAction.h>
#include <knowrob/askallAction.h>
#include <knowrob/tellAction.h>

typedef actionlib::SimpleActionClient<knowrob::askoneAction> askOneClient;
typedef actionlib::SimpleActionClient<knowrob::askincrementalAction> askIncrementalClient;
typedef actionlib::SimpleActionClient<knowrob::askallAction> askAllClient;
typedef actionlib::SimpleActionClient<knowrob::tellAction> tellClient;

typedef knowrob::GraphAnswerMessage KnowrobAnswer;
typedef knowrob::GraphQueryMessage KnowrobQuery;

class KnowrobClient
{
private:
    std::string m_logger_prefix;
    bool m_is_initialized;
    bool m_verbose;

    double m_query_timeout;

    std::shared_ptr<askOneClient> m_actCli_ask_one;
    std::shared_ptr<tellClient> m_actCli_tell;

    const knowrob::GraphQueryMessage m_default_query;

public:
    KnowrobClient();
    //~KnowrobClient();


    bool initialize(ros::NodeHandle &nh);

    KnowrobQuery getDefaultQueryMessage() const; 

    KnowrobQuery createQuery(const std::string &query,
                                           const std::string &lang = "",
                                           const int epistemic_operator = 0,
                                           const std::string &about_agent_iri = "",
                                           const std::string &about_simulation_iri = "",
                                           const int temporal_operator = 0,
                                           const double min_past_timestamp = 0.0,
                                           const double max_past_timestamp = 0.0,
                                           const double confidence = 0.0) const;

    bool askOne(const KnowrobQuery &knowrob_query,
                KnowrobAnswer &knowrob_answer);

    void askIncremental();
    void askAll();

    bool tell(const KnowrobQuery &knowrob_query);
};
