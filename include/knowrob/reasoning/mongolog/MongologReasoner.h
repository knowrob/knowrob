/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_MONGOLOG_REASONER_H__
#define __KNOWROB_MONGOLOG_REASONER_H__

#include "knowrob/reasoning/LogicProgramReasoner.h"
#include "knowrob/reasoning/mongolog/MongologFactBase.h"
#include "knowrob/reasoning/mongolog/MongologRuleBase.h"

namespace knowrob {
    class MongologReasoner : public LogicProgramReasoner<MongologFactBase,MongologRuleBase> {
    public:
        MongologReasoner(
            std::shared_ptr<MongologFactBase> &edb,
            std::shared_ptr<MongologRuleBase> &idb)
            : LogicProgramReasoner(edb, idb) {};
        ~MongologReasoner() {};

    protected:
    };
}

#endif //__KNOWROB_MONGOLOG_REASONER_H__
