/*
 * Copyright (c) 2023, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// STD
#include <memory>
// KnowRob
#include <knowrob/logging.h>
#include <knowrob/esg/ESGReasoner.h>

using namespace knowrob;

ESGReasoner::ESGReasoner(const std::string &reasonerID)
: PrologReasoner(reasonerID)
{
}

bool ESGReasoner::initializeDefaultPackages()
{
	return consult(std::filesystem::path("esg") / "__init__.pl");
}

// A fixture for testing.
class ESGTests: public PrologTests {
protected:
	static std::shared_ptr<knowrob::ESGReasoner> reasoner() {
		static std::shared_ptr<knowrob::ESGReasoner> r;
		if(!r) {
			KB_INFO("Running ESG tests");
			r = std::make_shared<knowrob::ESGReasoner>("esg0");
			r->loadConfiguration(knowrob::ReasonerConfiguration());
		}
		return r;
	}

	// Per-test-suite set-up.
	static void SetUpTestSuite() { reasoner();  }

	static std::string getPath(const std::string &relativePath) {
		// TODO: proper handling of paths
		return std::string("src/esg/") + relativePath;
	}
};

#define ESG_TEST_F(NAME, FILE) TEST_F(ESGTests, NAME) { runPrologTests(reasoner(), getPath(FILE)); }

ESG_TEST_F(esg,			"esg.plt")
ESG_TEST_F(interval,	"interval.plt")
ESG_TEST_F(workflow,	"workflow.pl")
//ESG_TEST_F(parser,	"parser.plt")
