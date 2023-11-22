#include <knowrob/ros_client/cpp/knowrob_client.hpp>

KnowrobClient::KnowrobClient() : 
    m_logger_prefix("[KR-RC]\t"),
    m_is_initialized(false),
    m_verbose(false),
    m_query_timeout(0.0)
{
    ros::NodeHandle prvt_nh("~");
    prvt_nh.param("logger_prefix", m_logger_prefix, std::string("[KR-RC]\t"));
    prvt_nh.param("verbose", m_verbose, false);

    ROS_INFO_STREAM_COND(m_verbose, m_logger_prefix << "Verbose enabled!");
}

bool KnowrobClient::initialize(ros::NodeHandle &nh)
{
    ros::NodeHandle prvt_nh("~");
    std::string action_topic;

    prvt_nh.param("query_timeout", m_query_timeout, 0.0);
    ROS_INFO_STREAM_COND(m_verbose, m_logger_prefix << "Query timeout is set to " << m_query_timeout << "s");


    // ASK ONE
    prvt_nh.param("askAll/action_topic", action_topic, std::string("askAll"));
    m_actCli_ask_one = std::make_shared<askOneClient>(nh, action_topic, true);

    bool wait_for_actionserver = false;
    prvt_nh.param("askAll/wait_for_actionserver", wait_for_actionserver, false);

    if (wait_for_actionserver)
    {
        ROS_INFO_STREAM(m_logger_prefix << "Waiting for actionserver to start.. Topic = " << action_topic);
        m_actCli_ask_one->waitForServer();
        ROS_INFO_STREAM(m_logger_prefix << "Server is ready!");
    }

    // ASK I

    // ASK ALL

    // TELL

    ROS_INFO_STREAM(m_logger_prefix << "Init complete!");
    return true;
}

bool KnowrobClient::askOne(const std::string& query, 
                const std::string& lang,
                const int epistemic_operator,
                const std::string& about_agent_iri,
                const std::string& about_simulation_iri,
                const int temporal_operator,
                const double min_past_timestamp,
                const double max_past_timestamp,
                const double confidence)
{
    // pre-check if server is available
    if(!m_is_initialized)
    {
        ROS_ERROR_STREAM(m_logger_prefix << "Knowrob client is not initialized! Call initialize() function first!");
        return false;
    }

    if (!m_actCli_ask_one->isServerConnected())
    {
        ROS_ERROR_STREAM(m_logger_prefix << "Ask One - Actionserver is not connected!");
        return false;
    }

    // cancel all remaining goals
    m_actCli_ask_one->cancelAllGoals();

    // build the action goal
    knowrob::askoneGoal action_goal;

    action_goal.query.queryString = query;
    action_goal.query.lang = lang;
    action_goal.query.epistemicOperator = epistemic_operator;
    action_goal.query.aboutAgentIRI = about_agent_iri;
    action_goal.query.aboutSimulationIRI = about_simulation_iri;
    action_goal.query.temporalOperator = temporal_operator;
    action_goal.query.minPastTimestamp = min_past_timestamp;
    action_goal.query.maxPastTimestamp = max_past_timestamp;
    action_goal.query.confidence = confidence;

    // send the goal and wait for an answer
    m_actCli_ask_one->sendGoal(action_goal);

    if(m_actCli_ask_one->waitForResult())
    {
        ROS_INFO_STREAM_COND(m_verbose, m_logger_prefix << "Action finished!");
    }   
    else
    {
        ROS_ERROR_STREAM(m_logger_prefix << "Timeout for askone action!");
        return false;
    }

    // inspect the result
    knowrob::askoneResultConstPtr action_result = m_actCli_ask_one->getResult();

    

    return true;
}