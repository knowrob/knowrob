/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TF_REASONER_H
#define KNOWROB_TF_REASONER_H

// KnowRob
#include <knowrob/reasoner/prolog/PrologReasoner.h>

namespace knowrob {

	class TFReasoner : public PrologReasoner {
	public:
		static const std::string SWRL_FORMAT;

		explicit TFReasoner(const std::string &reasonerID);

		bool loadSWRLFile(const DataSourcePtr &dataFile);

	protected:
		// Override PrologReasoner
		bool initializeDefaultPackages() override;
	};

} // knowrob

#endif //KNOWROB_TF_REASONER_H
