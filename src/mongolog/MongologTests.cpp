/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include <knowrob/logging.h>
#include <knowrob/mongolog/MongologReasoner.h>

using namespace knowrob;

// A fixture for testing mongolog.
class MongologTests: public PrologTests {
protected:
	static std::shared_ptr<knowrob::MongologReasoner> reasoner() {
		static std::shared_ptr<knowrob::MongologReasoner> r;
		if(r.get()==nullptr) {
			KB_INFO("Running Mongolog tests");
			knowrob::ReasonerConfiguration reasonerConfig;
			r = std::make_shared<knowrob::MongologReasoner>("mongolog0");
			r->loadConfiguration(reasonerConfig);
		}
		return r;
	}

	// Per-test-suite set-up.
	// Called before the first test in this test suite.
	static void SetUpTestSuite() { reasoner();  }

	static std::string mongologPath(const std::string &relativePath) {
		// TODO: proper handling of paths
		return std::string("src/mongolog/") + relativePath;
	}

	static void runMongologTests(const std::string target) {
		runPrologTests(reasoner(), target);
	}
};

#define MONGOLOG_TEST_F(NAME, MONGOLOG_FILE) TEST_F(MongologTests, NAME) { \
	runMongologTests(mongologPath(MONGOLOG_FILE));}

MONGOLOG_TEST_F(arithmetic,		"arithmetic.pl")
MONGOLOG_TEST_F(atoms,			"atoms.pl")
MONGOLOG_TEST_F(comparison,		"comparison.pl")
MONGOLOG_TEST_F(control,		"control.pl")
MONGOLOG_TEST_F(database,		"database.pl")
MONGOLOG_TEST_F(findall,		"findall.pl")
MONGOLOG_TEST_F(fluents,		"fluents.pl")
MONGOLOG_TEST_F(lists,			"lists.pl")
MONGOLOG_TEST_F(meta,			"meta.pl")
MONGOLOG_TEST_F(sgml,			"sgml.pl")
MONGOLOG_TEST_F(terms,			"terms.pl")
MONGOLOG_TEST_F(typecheck,		"typecheck.pl")
MONGOLOG_TEST_F(unification,	"unification.pl")
MONGOLOG_TEST_F(rdfs,			"rdfs.pl")
MONGOLOG_TEST_F(annotation,		"annotation.pl")
MONGOLOG_TEST_F(triple,			"triple.plt")
MONGOLOG_TEST_F(holds,			"holds.pl")
MONGOLOG_TEST_F(temporal,		"temporal.pl")
