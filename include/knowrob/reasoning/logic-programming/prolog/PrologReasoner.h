/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PROLOG_REASONER_H__
#define __KNOWROB_PROLOG_REASONER_H__

#include "knowrob/reasoning/logic-programming/LogicProgramReasoner.h"

namespace knowrob {
    class PrologReasoner : public LogicProgramReasoner {
    public:
        PrologReasoner();
        ~PrologReasoner();

    private:
    };
}

#endif //__KNOWROB_PROLOG_REASONER_H__
