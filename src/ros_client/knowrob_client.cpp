#include <knowrob/ros_client/cpp/knowrob_client.hpp>

KnowrobClient::KnowrobClient() : 
    m_logger_prefix("[KR-RC]\t"),
    m_is_initialized(false),
    m_verbose(false),
    m_query_timeout(0.0)
{
    ros::NodeHandle prvt_nh("~");
    prvt_nh.param("logger_prefix", m_logger_prefix, std::string("[KR-RC]\t"));
    prvt_nh.param("verbose", m_verbose, true);

    ROS_INFO_STREAM_COND(m_verbose, m_logger_prefix << "Verbose enabled!");
}

bool KnowrobClient::initialize(ros::NodeHandle &nh)
{
    ROS_INFO_STREAM_COND(m_verbose, m_logger_prefix << "Initializing client interfaces..");

    ros::NodeHandle prvt_nh("~");
    std::string action_topic;

    prvt_nh.param("query_timeout", m_query_timeout, 0.0);
    ROS_INFO_STREAM_COND(m_verbose, m_logger_prefix << "Query timeout is set to " << m_query_timeout << "s");

    double action_server_timeout = 0.0;
    prvt_nh.param("action_server_timeout", action_server_timeout, 1.0);
    ros::Duration dur_action_server_timeout(action_server_timeout);

    // ASK ONE
    prvt_nh.param("ask_one/action_topic", action_topic, std::string("knowrob/askone"));
    m_actCli_ask_one = std::make_shared<askOneClient>(nh, action_topic, true);

    bool wait_for_actionserver = false;
    prvt_nh.param("ask_one/wait_for_actionserver", wait_for_actionserver, true);

    if (wait_for_actionserver)
    {
        ROS_INFO_STREAM(m_logger_prefix << "Waiting for actionserver to start.. Topic = " << action_topic);
        if(m_actCli_ask_one->waitForServer(dur_action_server_timeout))
        {
            ROS_INFO_STREAM(m_logger_prefix << "Server is ready!");
        }
        else
        {
            ROS_ERROR_STREAM(m_logger_prefix << "Timeout while waiting for the ask one action server!"); 
            return false;
        }
        ROS_INFO_STREAM(m_logger_prefix << "Server is ready!");
    }   

    // ASK ALL
    prvt_nh.param("ask_all/action_topic", action_topic, std::string("knowrob/askall"));
    m_actCli_ask_all = std::make_shared<askAllClient>(nh, action_topic, true);

    wait_for_actionserver = false;
    prvt_nh.param("ask_all/wait_for_actionserver", wait_for_actionserver, true);

    if (wait_for_actionserver)
    {
        ROS_INFO_STREAM(m_logger_prefix << "Waiting for actionserver to start.. Topic = " << action_topic);
        if(m_actCli_ask_all->waitForServer(dur_action_server_timeout))
        {
            ROS_INFO_STREAM(m_logger_prefix << "Server is ready!");
        }
        else
        {
            ROS_ERROR_STREAM(m_logger_prefix << "Timeout while waiting for the ask all action server!"); 
            return false;
        }
        ROS_INFO_STREAM(m_logger_prefix << "Server is ready!");
    }

    // ASK INCREMENTAL

    // BLANK, is harder to implement as feedback is needed



    // TELL
    prvt_nh.param("tell/action_topic", action_topic, std::string("knowrob/tell"));
    m_actCli_tell = std::make_shared<tellClient>(nh, action_topic, true);

    wait_for_actionserver = false;
    prvt_nh.param("tell/wait_for_actionserver", wait_for_actionserver, true);

    if (wait_for_actionserver)
    {
        ROS_INFO_STREAM(m_logger_prefix << "Waiting for actionserver to start.. Topic = " << action_topic);
        if(m_actCli_tell->waitForServer(dur_action_server_timeout))
        {
            ROS_INFO_STREAM(m_logger_prefix << "Server is ready!");
        }
        else
        {
            ROS_ERROR_STREAM(m_logger_prefix << "Timeout while waiting for the tell action server!"); 
        }
    }

    ROS_INFO_STREAM(m_logger_prefix << "Init complete!");
    m_is_initialized = true;
    return true;
}


KnowrobQuery KnowrobClient::getDefaultQueryMessage() const
{ 
    return m_default_query; 
}

KnowrobQuery KnowrobClient::createQuery(const std::string &query,
                                           const std::string &lang,
                                           const int epistemic_operator,
                                           const std::string &about_agent_iri,
                                           const std::string &about_simulation_iri,
                                           const int temporal_operator,
                                           const double min_past_timestamp,
                                           const double max_past_timestamp,
                                           const double confidence) const
{
    KnowrobQuery gqm;
    
    gqm.queryString = query;
    gqm.lang = lang;
    gqm.epistemicOperator = epistemic_operator;
    gqm.aboutAgentIRI = about_agent_iri;
    gqm.aboutSimulationIRI = about_simulation_iri;
    gqm.temporalOperator = temporal_operator;
    gqm.minPastTimestamp = min_past_timestamp;
    gqm.maxPastTimestamp = max_past_timestamp;
    gqm.confidence = confidence;

    return gqm;
}


std::string KnowrobClient::getAnswerText(const KnowrobAnswer& answer) const
{
    std::stringstream ss;

    int i = 0;
    for(auto& elem : answer.substitution)
    {
        ss << "[" << i << "]" << "\t" 
           << "[Key]=" << elem.key << "\t"
           << "[Value]=";

        switch(elem.type)
        {
            case knowrob::KeyValuePair::TYPE_STRING:
            {
                ss << elem.value_string;
            }
            break;
            case knowrob::KeyValuePair::TYPE_FLOAT:
            {
                ss << elem.value_float;
            }
            break;
            case knowrob::KeyValuePair::TYPE_INT:
            {
                ss << elem.value_int;
            }
            break;
            case knowrob::KeyValuePair::TYPE_LONG:
            {
                ss << elem.value_long;
            }
            break;
            case knowrob::KeyValuePair::TYPE_VARIABLE:
            {
                ss << elem.value_variable;
            }
            break;
            case knowrob::KeyValuePair::TYPE_PREDICATE:
            {
                ss << elem.value_predicate;
            }
            break;
            case knowrob::KeyValuePair::TYPE_LIST:
            {
                ss << elem.value_list;
            }
            break;    
            default:
                ROS_ERROR_STREAM(m_logger_prefix << "Unknown Answer Substitution Type!");
                return std::string("ERROR");
                break;
        }
    
        ss << std::endl;
    }
    
    return ss.str();
}




bool KnowrobClient::askOne(const KnowrobQuery &knowrob_query,
                           KnowrobAnswer &knowrob_answer)
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
    action_goal.query = knowrob_query;
 
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

    knowrob_answer = action_result->answer;

    return true;
}



bool KnowrobClient::askAll(const KnowrobQuery &knowrob_query,
                           std::vector<KnowrobAnswer>& knowrob_answers)
{
    // pre-check if server is available
    if(!m_is_initialized)
    {
        ROS_ERROR_STREAM(m_logger_prefix << "Knowrob client is not initialized! Call initialize() function first!");
        return false;
    }

    if (!m_actCli_ask_all->isServerConnected())
    {
        ROS_ERROR_STREAM(m_logger_prefix << "Ask All - Actionserver is not connected!");
        return false;
    }

    // cancel all remaining goals
    m_actCli_ask_all->cancelAllGoals();

    // build the action goal
    knowrob::askallGoal action_goal;
    action_goal.query = knowrob_query;
 
    // send the goal and wait for an answer
    m_actCli_ask_all->sendGoal(action_goal);

    if(m_actCli_ask_all->waitForResult())
    {
        ROS_INFO_STREAM_COND(m_verbose, m_logger_prefix << "Action finished!");
    }   
    else
    {
        ROS_ERROR_STREAM(m_logger_prefix << "Timeout for askall action!");
        return false;
    }

    // inspect the result
    knowrob::askallResultConstPtr action_result = m_actCli_ask_all->getResult();

    knowrob_answers = action_result->answer;

    if(!knowrob_answers.size())
    {
        ROS_ERROR_STREAM(m_logger_prefix << "Ask All - Answers vector is empty!");
        return false;
    }

    return true;

}




bool KnowrobClient::tell(const KnowrobQuery &knowrob_query)
{
    // pre-check if server is available
    if(!m_is_initialized)
    {
        ROS_ERROR_STREAM(m_logger_prefix << "Knowrob client is not initialized! Call initialize() function first!");
        return false;
    }

    if (!m_actCli_ask_one->isServerConnected())
    {
        ROS_ERROR_STREAM(m_logger_prefix << "Tell - Actionserver is not connected!");
        return false;
    }

    // cancel all remaining goals
    m_actCli_tell->cancelAllGoals();

    // build the action goal
    knowrob::tellGoal action_goal;
    action_goal.query = knowrob_query;
 
    // send the goal and wait for an answer
    m_actCli_tell->sendGoal(action_goal);

    if(m_actCli_tell->waitForResult())
    {
        ROS_INFO_STREAM_COND(m_verbose, m_logger_prefix << "Action finished!");
    }   
    else
    {
        ROS_ERROR_STREAM(m_logger_prefix << "Timeout for askone action!");
        return false;
    }

    // inspect the result
    knowrob::tellResultConstPtr action_result = m_actCli_tell->getResult();

    if(action_result->status == action_result->TELL_FAILED)
    {
        ROS_ERROR_STREAM(m_logger_prefix << "Tell failed!");
        return false;
    }

    return true;
}
