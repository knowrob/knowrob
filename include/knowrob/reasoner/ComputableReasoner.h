/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_IMPLEMENTED_REASONER_H_
#define KNOWROB_IMPLEMENTED_REASONER_H_

#include <set>
#include "Reasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"


namespace knowrob {

	using computableFunc = void (*)(const std::shared_ptr<knowrob::RDFLiteral> &literal,
									const std::shared_ptr<AnswerStream::Channel> &outputChannel);
	
	/**
	 * An interface for reasoning subsystems.
	 */
	class ComputableReasoner : public Reasoner {
	public:
		ComputableReasoner() = default;
		virtual ~ComputableReasoner()= default;


		// Override IReasoner
		std::shared_ptr<PredicateDescription> getPredicateDescription(
				const std::shared_ptr<PredicateIndicator> &indicator) override;

	protected:
		// Map of provided functors to implemented functions
		std::map<std::string, computableFunc> functors;
	};
}

#endif //KNOWROB_IMPLEMENTED_REASONER_H_
