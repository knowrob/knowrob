/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PROLOG_FACT_BASE_H__
#define __KNOWROB_PROLOG_FACT_BASE_H__

#include "knowrob/db/IFactBase.h"

namespace knowrob {
    class PrologFactBase : public IFactBase<Predicate> {
    public:
        PrologFactBase();
        ~PrologFactBase();

    protected:
    };
}

#endif //__KNOWROB_PROLOG_FACT_BASE_H__
