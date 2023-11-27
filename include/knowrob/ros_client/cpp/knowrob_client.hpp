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
    std::shared_ptr<askAllClient> m_actCli_ask_all;

    std::shared_ptr<tellClient> m_actCli_tell;

    const knowrob::GraphQueryMessage m_default_query;

public:
    KnowrobClient();
    //~KnowrobClient();

    /**
     * @brief Initializes parameters from ROS param server and connects the action servers.
     * Blocking if the action servers are not reachable
     * 
     * @param nh a nodehandle from your node
     * @return true if the process was finished successfully
     * @return false if something went wrong
     */
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

    std::string getAnswerText(const KnowrobAnswer& answer) const;

    /**
     * @brief 
     * 
     * @param knowrob_query the query you want to ask
     * @param knowrob_answer a single answer propt by knowrob
     * @return true when the call was successful - not considering the content of the answer
     * @return false when something went wrong during communication
     */
    bool askOne(const KnowrobQuery &knowrob_query,
                KnowrobAnswer &knowrob_answer);

    /**
     * @brief 
     * 
     * @param knowrob_query the query you want to ask
     * @param knowrob_answers an array of all answers from knowrob
     * @return true when the call was successful - not considering the content of the answer
     * @return false when something went wrong during communication
     */
    bool askAll(const KnowrobQuery &knowrob_query,
                std::vector<KnowrobAnswer> &knowrob_answers);

    void askIncremental();

    /**
     * @brief 
     * 
     * @param knowrob_query the query you want to tell
     * @return true when the call was successful - not considering the content of the answer
     * @return false when something went wrong during communication
     */
    bool tell(const KnowrobQuery &knowrob_query);
};
