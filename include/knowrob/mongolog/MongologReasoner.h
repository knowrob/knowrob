/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGOLOG_REASONER_H_
#define KNOWROB_MONGOLOG_REASONER_H_

// STD
#include <memory>
// KnowRob
#include "knowrob/terms/Term.h"
#include <knowrob/IReasoner.h>
#include <knowrob/prolog/PrologReasoner.h>

namespace knowrob {
	class MongologReasoner : public PrologReasoner {
	public:
		explicit MongologReasoner(const std::string &reasonerID);

		~MongologReasoner() override;
		
	protected:
		// Override PrologReasoner
		const functor_t& callFunctor() override;

		// Override PrologReasoner
		bool initializeDefaultPackages() override;
    };
}

#endif //KNOWROB_MONGOLOG_REASONER_H_
