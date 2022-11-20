/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_REASONER_MANAGER_H__
#define __KNOWROB_REASONER_MANAGER_H__

#include <list>
#include <memory>

#include "knowrob/lang/Predicate.h"
#include "knowrob/reasoning/IReasoner.h"
#include "knowrob/reasoning/ReasoningProcess.h"
#include "knowrob/reasoning/ReasoningTask.h"

namespace knowrob {
    /**
     * Manages the set of available reasoning subsystems.
     */
    class ReasonerManager {
    public:
        ReasonerManager();
        ~ReasonerManager();

        void addReasoner(std::shared_ptr<IReasoner> &reasoner);
        void removeReasoner(std::shared_ptr<IReasoner> &reasoner);

        /** Get list of reasoner that can handle given predicate.
         *
         * @param predicate the predicate in question
         * @return an essemble of reasoner that can handle the predicate
         */
        list<std::shared_ptr<IReasoner>> getExpertsForPredicate(const Predicate &predicate);

        /** Start a reasoning process for given reasoning task.
         *
         * Each process is executed in a separate thread managed by this thread pool.
         * The thread is released automatically when the reasoner finishes evaluation.
         * Until then, the pool keeps a reference on the shared pointer.
         * Depending on the particular IReasoner implementation the process may can be
         * stopped, paused and continued.
         *
         * @param task a reasoning task
         * @return the reasoning process started
         */
         std::shared_ptr<ReasoningProcess> submitTask(const ReasoningTask &task);

    private:
        list<std::shared_ptr<IReasoner>> pool_;
    };
}

#endif //__KNOWROB_REASONER_MANAGER_H__
