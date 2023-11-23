/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/prolog/PrologTests.h"
#include "knowrob/reasoner/tf/TFReasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"

using namespace knowrob;

// make reasoner type accessible
KNOWROB_BUILTIN_REASONER("TF", TFReasoner)

TFReasoner::TFReasoner(const std::string &reasonerID)
		: PrologReasoner(reasonerID)
{
    addDataSourceHandler(SWRL_FORMAT, [this]
            (const DataSourcePtr &dataFile) { return loadSWRLFile(dataFile); });
}

bool SWRLReasoner::loadSWRLFile(const DataSourcePtr &dataFile)
{
	static auto consult_f = std::make_shared<PredicateIndicator>("swrl_file_load", 1);
	auto path = getResourcePath(dataFile->uri());
	auto arg0 = std::make_shared<StringTerm>(path.native());
	return eval(std::make_shared<Predicate>(Predicate(consult_f, { arg0 })));
}

bool SWRLReasoner::initializeDefaultPackages()
{
	return consult(std::filesystem::path("reasoner") / "swrl" / "__init__.pl");
}

class SWRLTests: public PrologTests<knowrob::SWRLReasoner> {
protected:
	static std::string getPath(const std::string &filename)
	{ return std::filesystem::path("reasoner") / "swrl" / filename; }
};

TEST_F(SWRLTests, swrl) { runTests(getPath("swrl.plt")); }
