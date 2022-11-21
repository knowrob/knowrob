/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_BLACKBOARD_H__
#define __KNOWROB_BLACKBOARD_H__

#include <list>
// boost
#include <boost/shared_ptr.hpp>

#include "knowrob/lang/IQuery.h"
#include "knowrob/lang/Answer.h"
#include "knowrob/qa/AnswerPublisher.h"
#include "knowrob/qa/BlackboardSegment.h"
#include "knowrob/reasoning/ReasonerManager.h"

namespace knowrob {
    /**
     * A board where multiple experts can contribute in answering a query.
     */
    class Blackboard : public AnswerPublisher {
    public:
        Blackboard(
            const boost::shared_ptr<ReasonerManager> &reasonerManager,
            const boost::shared_ptr<IQuery> &goal);
        ~Blackboard();

        /** Get list of blackboard segments.
         *
         * Segments represent a decomposition of the goal where each subgoal is handled within a segment of the blackboard.
         * Note that the combination of different segments is also done in a dedicated segment of the blackboard.
         *
         * @return list of blackboard segments
         */
        const std::list<BlackboardSegment>& segments() const { return segments_; }

    private:
        boost::shared_ptr<ReasonerManager> reasonerManager_;
        std::list<BlackboardSegment> segments_;

        /** Decompose the blackboard into different segments. */
        void decompose();

        /** Stop all reasoning processes attached to segments. */
        void stopReasoningProcesses();
    };
}

#endif //__KNOWROB_BLACKBOARD_H__
