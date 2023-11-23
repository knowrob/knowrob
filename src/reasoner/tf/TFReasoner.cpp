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

TFReasoner::TFReasoner(const std::string &reasonerID)
		: PrologReasoner(reasonerID)
{
    bool loadConfiguration(const ReasonerConfiguration &cfg) override { return true; }
    void setDataBackend(const KnowledgeGraphPtr &knowledgeGraph) override {}
    unsigned long getCapabilities() const override { return CAPABILITY_TOP_DOWN_EVALUATION; };
}
