/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_OWL_REASONER_H__
#define __KNOWROB_OWL_REASONER_H__

#include "knowrob/reasoning/IReasoner.h"

namespace knowrob {
    class OWLReasoner : public IReasoner {
    public:
        OWLReasoner(
            std::shared_ptr<RDFTripleStore> &tripleStore,
            std::shared_ptr<OWLAxiomStore> &axiomStore)
            : tripleStore_(tripleStore), axiomStore_(axiomStore) {};
        ~OWLReasoner();

    private:
    };
}

#endif //__KNOWROB_OWL_REASONER_H__
