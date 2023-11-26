/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/tf/TFReasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"

using namespace knowrob;

// make reasoner type accessible
KNOWROB_BUILTIN_REASONER("TF", TFReasoner)

// Set of provided functors
std::set<std::string> functors = {"is_at", "tf_get_pose"};

TFReasoner::TFReasoner(std::string reasonerID)
{
}

bool loadConfiguration(const ReasonerConfiguration &cfg)  { return true; }
void setDataBackend(const KnowledgeGraphPtr &knowledgeGraph) {}
unsigned long getCapabilities() { return CAPABILITY_TOP_DOWN_EVALUATION; };

std::shared_ptr<PredicateDescription> getPredicateDescription(
        const std::shared_ptr<PredicateIndicator> &indicator) {
	if(functors.count(indicator->functor())) {
		return std::make_shared<PredicateDescription>(
			std::make_shared<PredicateIndicator>(indicator->functor(),2), PredicateType::RELATION);
	}
	else {
		return {};
	}
}
