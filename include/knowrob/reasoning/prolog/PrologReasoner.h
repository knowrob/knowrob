/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PROLOG_REASONER_H__
#define __KNOWROB_PROLOG_REASONER_H__

// STD
#include <string>
// boost
#include <boost/shared_ptr.hpp>
// KnowRob
#include <knowrob/reasoning/LogicProgramReasoner.h>
#include <knowrob/reasoning/ReasoningStatus.h>
#include <knowrob/reasoning/prolog/PrologFactBase.h>
#include <knowrob/reasoning/prolog/PrologRuleBase.h>
#include <knowrob/lang/Predicate.h>
#include <knowrob/lang/IQuery.h>

namespace knowrob {
    /**
     * A Prolog reasoner that performs reasoning using SWI Prolog.
     */
    class PrologReasoner : public LogicProgramReasoner {
    public:
        PrologReasoner(const std::string &initFile);
        ~PrologReasoner();

        /**
         * Consults a Prolog file, i.e. loads facts and rules and executed
         * directives in the file.
         * May throw an exception if there is no valid Prolog file at the given path.
         */
        void consult(const std:string &prologFile);

        // Override IReasoner::initialize
        virtual void initialize();

        // Override LogicProgramReasoner::initialize
        virtual void assert(const Predicate &predicate);

        // Override IReasoner::run
        virtual void runQuery(const IQuery &goal, ReasoningStatus &status, MessageQueue<Answer> &answerQueue);

        // Override IReasoner::canReasonAbout
        virtual bool canReasonAbout(const PredicateIndicator &predicate) const;

    protected:
        std::string initFile_;
    };
}

#endif //__KNOWROB_PROLOG_REASONER_H__
