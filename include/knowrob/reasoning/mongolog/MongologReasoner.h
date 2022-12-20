/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_MONGOLOG_REASONER_H__
#define __KNOWROB_MONGOLOG_REASONER_H__

// STD
#include <memory>
// KnowRob
#include <knowrob/lang/terms.h>
#include <knowrob/reasoning/IReasoner.h>
#include <knowrob/reasoning/prolog/PrologReasoner.h>

namespace knowrob {
	class MongologReasoner : public PrologReasoner {
	public:
		MongologReasoner();
		~MongologReasoner();
		
		// Override IReasoner
		bool initialize(const ReasonerConfiguration &cfg);
		
		// Override IReasoner
		bool canReasonAbout(const PredicateIndicator &predicate);
	protected:
		
		// Override PrologReasoner
		std::shared_ptr<Query> transformQuery(const std::shared_ptr<Query> &q);
    };
}

#endif //__KNOWROB_MONGOLOG_REASONER_H__
