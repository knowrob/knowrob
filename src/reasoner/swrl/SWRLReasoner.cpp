/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/prolog/PrologTests.h"
#include "knowrob/reasoner/swrl/SWRLReasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"

/*
	- TODO: make swrl configurable for mongolog
	- idea: 3 ways: 1. standalone prolog reasoner 2. datasource of prolog 3. datasource of mongolog
		- PrologReasoner: __init__.pl could be added as DataSource,
		  but then all c++ code would be bypassed (loading swrl format)
		  -> maybe file loader can be registered from prolog code, but would need
		     to interact with reasoner state, e.g. via static map of reasoner instances.
		- MongologReasoner: __init__.pl can also be loaded, database interface must be exchanged though.
		- can the PrologReasoner have sub-reasoner? hmm might not be a good idea
	- TODO: register format handler through prolog declarations
 		- I guess must be a multifile predicate that PrologReasoner
		  checks after package initialization. but more complex, there should
		  be format handlers available only to a subset of reasoner.
		  well could maybe be that init declares facts thatare loaded into  plugin modules
		  and cpp looks up these facts only from the plugin module
	- TODO: exchange swrl database interface via a flag. currently a reasoner_setting swrl:backend
	        is thought for that. but it seems a bit unelegant to check this flag always.
	        - options could be included as argument internally to make it less ugly.
	        - it could be ignored and the check done each and every time
	        - the database predicates could be loaded into the plugin module, and reasoner could use these instead of an explicit backend
 */

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
