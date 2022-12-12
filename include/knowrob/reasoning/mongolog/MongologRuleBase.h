/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_MONGOLOG_RULE_BASE_H__
#define __KNOWROB_MONGOLOG_RULE_BASE_H__

#include "knowrob/db/IRuleBase.h"

namespace knowrob {
    class MongologRuleBase : public IRuleBase {
    public:
        MongologRuleBase();
        ~MongologRuleBase();

    protected:
    };
}

#endif //__KNOWROB_MONGOLOG_RULE_BASE_H__
