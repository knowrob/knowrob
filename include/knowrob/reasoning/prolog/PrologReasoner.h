/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PROLOG_REASONER_H__
#define __KNOWROB_PROLOG_REASONER_H__

// boost
#include <boost/shared_ptr.hpp>
// KnowRob
#include <knowrob/reasoning/LogicProgramReasoner.h>
#include <knowrob/reasoning/prolog/PrologFactBase.h>
#include <knowrob/reasoning/prolog/PrologRuleBase.h>

namespace knowrob {
    /**
     * A Prolog reasoner that performs reasoning using SWI Prolog.
     */
    class PrologReasoner : public LogicProgramReasoner<PrologFactBase,PrologRuleBase> {
    public:
        PrologReasoner(boost::shared_ptr<IFactBase> &edb, boost::shared_ptr<IRuleBase> &idb);
        ~PrologReasoner();

        // Override IReasoner::initialize
        virtual void initialize();

        // Override IReasoner::canReasonAbout
        virtual bool canReasonAbout(const PredicateIndicator &predicate) const;

        // Override IReasoner::run
        virtual void run(const IQuery &goal, ReasoningStatus &status, MessageQueue<Answer> &answerQueue);

    private:
    };
}

#endif //__KNOWROB_PROLOG_REASONER_H__
