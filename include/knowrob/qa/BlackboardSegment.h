/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_BLACKBOARD_SEGMENT_H__
#define __KNOWROB_BLACKBOARD_SEGMENT_H__

#include "knowrob/qa/Query.h"
#include "knowrob/qa/Answer.h"
#include "knowrob/qa/AnswerPublisher.h"
#include "knowrob/reasoning/ReasonerEssemble.h"

namespace knowrob {
    /**
     * The segment of a blackboard where an essemble of experts is working
     * on solving a common task.
     */
    class BlackboardSegment : public AnswerPublisher {
    public:
        BlackboardSegment(
            const std::shared_ptr<ReasonerManager> &reasonerManager,
            const list<std::shared_ptr<IReasoner>> &experts,
            const std::shared_ptr<Query> &query);
        ~BlackboardSegment();

    private:
        std::shared_ptr<ReasonerManager> reasonerManager_;
        std::list<std::shared_ptr<ReasoningProcess>> processes_;

        /** Start all reasoning processes attached to this segment. */
        void startReasoningProcesses();
        /** Stop all reasoning processes attached to this segment. */
        void stopReasoningProcesses();
    };
}

#endif //__KNOWROB_BLACKBOARD_SEGMENT_H__
