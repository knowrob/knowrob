/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_LOGIC_PROGRAM_REASONER_H__
#define __KNOWROB_LOGIC_PROGRAM_REASONER_H__

#include <memory>

#include "knowrob/reasoning/IReasoner.h"
#include "knowrob/db/IFactBase.h"
#include "knowrob/db/IRuleBase.h"

namespace knowrob {
    template <class EDBType, class IDBType>
    class LogicProgramReasoner : public IReasoner {
    public:
        LogicProgramReasoner(std::shared_ptr<EDBType> &edb, std::shared_ptr<IDBType> &idb)
            : edb_(edb), idb_(idb) {};
        ~LogicProgramReasoner() {};

        /** Get the fact base of the logic program.
         *
         * @return the fact base
         */
        const EDBType& edb() const { return *edb_; }

        /** Get the fact base of the logic program.
         *
         * @return the fact base
         */
        const IDBType& idb() const { return *idb_; }

    protected:
        std::shared_ptr<EDBType> edb_;
        std::shared_ptr<IDBType> idb_;
    };
}

#endif //__KNOWROB_LOGIC_PROGRAM_REASONER_H__
