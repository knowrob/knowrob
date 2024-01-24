/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PROLOG_QUERY_RUNNER_H_
#define KNOWROB_PROLOG_QUERY_RUNNER_H_

// STD
#include <string>
#include <list>
#include <filesystem>
#include <map>
#include <memory>
#include <utility>
// gtest
#include <gtest/gtest.h>
// KnowRob
#include "knowrob/ThreadPool.h"
#include "knowrob/terms/Term.h"
#include "knowrob/reasoner/LogicProgramReasoner.h"
#include "PrologQuery.h"
#include "PrologThreadPool.h"

namespace knowrob {
    class PrologReasoner;

    /** A runner that evaluates a Prolog query.  */
    class PrologQueryRunner : public ThreadPool::Runner {
    public:
        /** a query request for a runner */
        struct Request {
            const uint32_t queryID;
            std::shared_ptr<StringTerm> queryModule;
            std::shared_ptr<const Query> goal;
			GraphSelector graphSelector;
            functor_t callFunctor;
            Request(const std::shared_ptr<const Query> &goal,
                    functor_t callFunctor,
                    const std::shared_ptr<StringTerm> &queryModule,
                   const GraphSelector &graphSelector,
                    uint32_t queryID=0)
                    : queryID(queryID),
                      queryModule(queryModule),
                      goal(goal),
                      graphSelector(graphSelector),
                      callFunctor(callFunctor) {};
        };

        PrologQueryRunner(PrologReasoner *reasoner,
                Request request,
                const std::shared_ptr<AnswerStream::Channel> &outputChannel,
                bool sendEOS=false);

        // Override Runner
        void run() override;

    protected:
        std::shared_ptr<AnswerStream::Channel> outputChannel_;
        PrologReasoner *reasoner_;
        Request request_;
        bool sendEOS_;

        term_t createQueryArgumentTerms(PrologQuery &pl_goal, term_t solutionScopeVar, term_t predicatesVar);
        term_t createContextTerm(term_t solutionScopeVar, term_t predicatesVar);

        std::list<std::shared_ptr<PrologQueryRunner>>::iterator requestIterator;
        friend class PrologReasoner;
    };
}

#endif //KNOWROB_PROLOG_QUERY_RUNNER_H_
