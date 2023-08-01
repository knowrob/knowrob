/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */


#ifndef KNOWROB_ROSINTERFACE_H
#define KNOWROB_ROSINTERFACE_H

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
#include <actionlib/server/simple_action_server.h>

namespace knowrob {
    class ROSInterface {
    private:
        ros::NodeHandle nh_;

        // Action Servers
        actionlib::SimpleActionServer <iai_knowledge_msgs::askallAction> askall_action_server_;
        //actionlib::SimpleActionServer <iai_knowledge_msgs::askoneAction> askone_action_server_;
        // actionlib::SimpleActionServer <iai_knowledge_msgs::askiterativeAction> askiterative_action_server_;
        KnowledgeBase kb_;
    public:
        explicit ROSInterface(boost::property_tree::ptree ptree);

        virtual ~ROSInterface();

        void executeAskAllCB(const iai_knowledge_msgs::askallGoalConstPtr &goal);

        //void executeAskOneCB(const iai_knowledge_msgs::askoneGoalConstPtr &goal);

        //void executeAskIterativeCB(const iai_knowledge_msgs::askiterativeGoalConstPtr &goal);
        static ModalityFrame genModalityFrame(const iai_knowledge_msgs::askallGoal_<std::allocator<void>>::_query_type& query);

        iai_knowledge_msgs::KeyValuePair createKeyValuePair(AnswerPtr sharedPtr);

        iai_knowledge_msgs::GraphAnswer createGraphAnswer(std::shared_ptr<const Answer> sharedPtr);
    };
}

#endif //KNOWROB_ROSINTERFACE_H
