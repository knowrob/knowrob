/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/ComputableReasoner.h"


namespace knowrob {

	std::shared_ptr<PredicateDescription> ComputableReasoner::getPredicateDescription(
			const std::shared_ptr<PredicateIndicator> &indicator) {
		if(functors.count(indicator->functor())) {
			return std::make_shared<PredicateDescription>(
					std::make_shared<PredicateIndicator>(indicator->functor(),2), PredicateType::RELATION);
		}
		else {
			return {};
		}
	}



} // knowrob