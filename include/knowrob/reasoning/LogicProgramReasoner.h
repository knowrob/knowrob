/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_LOGIC_PROGRAM_REASONER_H__
#define __KNOWROB_LOGIC_PROGRAM_REASONER_H__

// boost
#include <boost/shared_ptr.hpp>
// KnowRob
#include <knowrob/reasoning/IReasoner.h>
#include <knowrob/db/IFactBase.h>
#include <knowrob/db/IRuleBase.h>

namespace knowrob {
    class LogicProgramReasoner : public IReasoner {
    public:
        LogicProgramReasoner() {}
        ~LogicProgramReasoner() {}

        /** Assert a fact into the reasoning system.
         *
         * @param predicate a fact
         */
        virtual void assert(const Predicate &predicate) = 0;

        /** Get the fact bases of the logic program.
         *
         * @return the fact bases
         */
        const std::list<boost::shared_ptr<IFactBase>>& edbs() const { return edbs_; }

        /** Get the fact bases of the logic program.
         *
         * @return the fact bases
         */
        const std::list<boost::shared_ptr<IRuleBase>>& idbs() const { return idbs_; }

        void addEDB(boost::shared_ptr<IFactBase> &edb) { edbs_.push_back(edb); }
        void addIDB(boost::shared_ptr<IRuleBase> &idb) { idbs_.push_back(idb); }

    protected:
        std::list<boost::shared_ptr<IFactBase>> edbs_;
        std::list<boost::shared_ptr<IRuleBase>> idbs_;
    };
}

#endif //__KNOWROB_LOGIC_PROGRAM_REASONER_H__
