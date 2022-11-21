/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_COMPUTATION_METHOD_H__
#define __KNOWROB_COMPUTATION_METHOD_H__

namespace knowrob {
    class IComputationMethod : public IReasoner{
    public:
        IComputationMethod();
        ~IComputationMethod();

    private:
    };
}

#endif //__KNOWROB_COMPUTATION_METHOD_H__
