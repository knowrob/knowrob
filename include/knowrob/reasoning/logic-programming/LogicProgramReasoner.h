/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_LOGIC_PROGRAM_REASONER_H__
#define __KNOWROB_LOGIC_PROGRAM_REASONER_H__

#include "knowrob/reasoning/IReasoner.h"

namespace knowrob {
    class LogicProgramReasoner : public IReasoner {
    public:
        LogicProgramReasoner();
        ~LogicProgramReasoner();

    private:
    };
}

#endif //__KNOWROB_LOGIC_PROGRAM_REASONER_H__
