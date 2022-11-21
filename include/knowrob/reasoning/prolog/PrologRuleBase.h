/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PROLOG_RULE_BASE_H__
#define __KNOWROB_PROLOG_RULE_BASE_H__

#include "knowrob/db/IRuleBase.h"

namespace knowrob {
    class PrologRuleBase : public IRuleBase {
    public:
        PrologRuleBase();
        ~PrologRuleBase();

    protected:
    };
}

#endif //__KNOWROB_PROLOG_RULE_BASE_H__
