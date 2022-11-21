/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PROLOG_REASONER_H__
#define __KNOWROB_PROLOG_REASONER_H__

#include "knowrob/reasoning/LogicProgramReasoner.h"
#include "knowrob/reasoning/prolog/PrologFactBase.h"
#include "knowrob/reasoning/prolog/PrologRuleBase.h"

namespace knowrob {
    class PrologReasoner : public LogicProgramReasoner<PrologFactBase,PrologRuleBase> {
    public:
        PrologReasoner(
            std::shared_ptr<PrologFactBase> &edb,
            std::shared_ptr<PrologRuleBase> &idb)
            : LogicProgramReasoner(edb, idb) {};
        ~PrologReasoner() {};

    private:
    };
}

#endif //__KNOWROB_PROLOG_REASONER_H__
