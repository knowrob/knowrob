/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_REASONING_TASK_H__
#define __KNOWROB_REASONING_TASK_H__

#include "knowrob/reasoning/IReasoner.h"
#include "knowrob/qa/Query.h"
#include "knowrob/qa/AnswerPublisher.h"

namespace knowrob {
    /**
     * The task of a reasoner to answer a certain query.
     */
    class ReasoningTask {
    public:
        ReasoningTask(
            std::shared_ptr<IReasoner> &reasoner,
            std::shared_ptr<IQuery> &goal,
            std::shared_ptr<IAnswerPublisher> &publisher)
            : reasoner_(reasoner), goal_(goal), publisher_(publisher) {};

        /** Get the reasoner associated to this task.
         *
         * @return the reasoner associated to this task
         */
        const IReasoner& reasoner() const { return *reasoner_; }

        /** Get the goal associated to this task.
         *
         * @return the goal associated to this task
         */
        const IQuery& goal() const { return *goal_; }

        /** Get the output stream associated to this task.
         *
         * @return the output stream associated to this task
         */
        const IAnswerPublisher& publisher() const { return *publisher_; }

    protected:
        std::shared_ptr<IReasoner> reasoner_;
        std::shared_ptr<IQuery> goal_;
        std::shared_ptr<IAnswerPublisher> publisher_;
    };
}

#endif //__KNOWROB_REASONING_TASK_H__
