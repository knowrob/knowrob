/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_MONGOLOG_FACT_BASE_H__
#define __KNOWROB_MONGOLOG_FACT_BASE_H__

#include "knowrob/db/IFactBase.h"

namespace knowrob {
    class MongologFactBase : public IFactBase<Predicate> {
    public:
        MongologFactBase();
        ~MongologFactBase();

    protected:
    };
}

#endif //__KNOWROB_MONGOLOG_FACT_BASE_H__
