/*
 * Copyright (c) 2023, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ESGREASONER_H
#define KNOWROB_ESGREASONER_H

// KnowRob
#include "knowrob/reasoner/prolog/PrologReasoner.h"

namespace knowrob {

	class ESGReasoner : public PrologReasoner {
	public:
		ESGReasoner();

	protected:
		// Override PrologReasoner
		bool initializeDefaultPackages() override;
	};

} // knowrob

#endif //KNOWROB_ESGREASONER_H
