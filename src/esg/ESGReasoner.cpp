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

class ESGTests: public PrologTests<knowrob::ESGReasoner> {
protected:
	static std::string getPath(const std::string &filename)
	{ return std::filesystem::path("esg") / filename; }
};

TEST_F(ESGTests, esg)      { runTests(getPath("esg.plt")); }
TEST_F(ESGTests, interval) { runTests(getPath("interval.plt")); }
TEST_F(ESGTests, workflow) { runTests(getPath("workflow.pl")); }
TEST_F(ESGTests, parser)   { runTests(getPath("parser.plt")); }
