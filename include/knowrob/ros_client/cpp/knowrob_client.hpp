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

#include <knowrob/askoneAction.h>
#include <knowrob/askincrementalAction.h>
#include <knowrob/askallAction.h>
#include <knowrob/tellAction.h>

typedef actionlib::SimpleActionClient<knowrob::askoneAction> askOneClient;
typedef actionlib::SimpleActionClient<knowrob::askincrementalAction> askIncrementalClient;
typedef actionlib::SimpleActionClient<knowrob::askallAction> askAllClient;
typedef actionlib::SimpleActionClient<knowrob::tellAction> tellClient;

class KnowrobClient
{
private:
    std::string m_logger_prefix;
    bool m_is_initialized;
    bool m_verbose;

    double m_query_timeout;

    std::shared_ptr<askOneClient> m_actCli_ask_one;

public:
    KnowrobClient();
    //~KnowrobClient();

    bool initialize(ros::NodeHandle& nh);

    bool askOne(const std::string& query, 
                const std::string& lang = "",
                const int epistemic_operator = 0,
                const std::string& about_agent_iri = "",
                const std::string& about_simulation_iri = "",
                const int temporal_operator = 0,
                const double min_past_timestamp = 0.0,
                const double max_past_timestamp = 0.0,
                const double confidence = 0.0);
    void askIncremental();
    void askAll();
    void tell();
};


