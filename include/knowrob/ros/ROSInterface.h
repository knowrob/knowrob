/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */


#ifndef KNOWROB_ROSINTERFACE_H
#define KNOWROB_ROSINTERFACE_H

#include "knowrob/interfaces/Interface.h"
// KnowRob
#include "knowrob/knowrob.h"
#include "knowrob/Logger.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/formulas/ModalFormula.h"
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <knowrob/GraphAnswerMessage.h>
#include <knowrob/GraphQueryMessage.h>
#include <knowrob/KeyValuePair.h>
#include <knowrob/askallAction.h>
#include <knowrob/askoneAction.h>
#include <knowrob/askincrementalAction.h>
#include <knowrob/tellAction.h>
#include <actionlib/server/simple_action_server.h>

namespace knowrob {
    class ROSInterface : public Interface {
    private:
        ros::NodeHandle nh_;

        // Action Servers
        actionlib::SimpleActionServer <askallAction> askall_action_server_;
        actionlib::SimpleActionServer <askoneAction> askone_action_server_;
        actionlib::SimpleActionServer <askincrementalAction> askincremental_action_server_;
        actionlib::SimpleActionServer <tellAction> tell_action_server_;
        KnowledgeBase kb_;
    public:
        explicit ROSInterface(const boost::property_tree::ptree& ptree);

        virtual ~ROSInterface();

        void executeAskAllCB(const askallGoalConstPtr &goal);

        void executeAskOneCB(const askoneGoalConstPtr &goal);

        void executeAskIncrementalCB(const askincrementalGoalConstPtr &goal);

        void executeTellCB(const tellGoalConstPtr &goal);

        static FormulaPtr
        applyModality(const GraphQueryMessage &query,
                      FormulaPtr ptr);

        GraphAnswerMessage createGraphAnswer(std::shared_ptr<const Answer> sharedPtr);
    };
}

#endif //KNOWROB_ROSINTERFACE_H
