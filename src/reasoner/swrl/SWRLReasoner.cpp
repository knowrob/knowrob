/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/prolog/PrologTests.h"
#include "knowrob/reasoner/swrl/SWRLReasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"

using namespace knowrob;

// make reasoner type accessible
KNOWROB_BUILTIN_REASONER("SWRL", SWRLReasoner)

const std::string SWRLReasoner::SWRL_FORMAT="pl-swrl";

SWRLReasoner::SWRLReasoner()
		: PrologReasoner()
{
	addDataHandler(SWRL_FORMAT, [this]
			(const DataSourcePtr &dataFile) { return loadSWRLFile(dataFile); });
}

bool SWRLReasoner::loadSWRLFile(const DataSourcePtr &dataFile)
{
	static auto consult_f = "swrl_file_load";
	auto path = PrologEngine::getResourcePath(dataFile->uri());
	KB_INFO("Loading SWRL file: {}", dataFile->uri());
	return PROLOG_ENGINE_EVAL(PrologTerm(consult_f, path.native()));
}

bool SWRLReasoner::initializeDefaultPackages()
{
	return consult(std::filesystem::path("reasoner") / "swrl" / "__init__.pl");
}

namespace knowrob::testing {
	class SWRLTests : public PrologTests<knowrob::SWRLReasoner, knowrob::PrologBackend> {
	protected:
		static std::string getPath(const std::string &filename) {
			return std::filesystem::path("reasoner") / "swrl" / filename;
		}
	};
}
using namespace knowrob::testing;

TEST_F(SWRLTests, swrl) { runTests(getPath("swrl.plt")); }
