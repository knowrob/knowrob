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

MongologReasoner::MongologReasoner(const std::string &reasonerID)
: PrologReasoner(reasonerID)
{}

MongologReasoner::~MongologReasoner()
= default;

bool MongologReasoner::initializeDefaultPackages()
{
	static bool initialized = false;

	if(initialized) {
		return true;
	} else {
		initialized = true;
		// load mongolog code once globally into the Prolog engine
		return consult(std::filesystem::path("mongolog") / "__init__.pl", "user", false);
	}
}

bool MongologReasoner::isCurrentPredicate(const PredicateIndicator &predicate)
{
	return eval(std::make_shared<Predicate>(Predicate(
			"current_predicate", {
				std::make_shared<StringTerm>(predicate.functor()) //,
				//std::make_shared<Integer32Term>(predicate.arity())
			}
	)));
}

std::shared_ptr<Query> MongologReasoner::transformQuery(const std::shared_ptr<Query> &q)
{
	static const auto indicator = std::make_shared<PredicateIndicator>("mongolog_call",1);
	
	// wrap queries in `mongolog_call/1`
	auto t = PrologQuery::toTerm(q->formula());
	return std::make_shared<Query>(
		std::make_shared<Predicate>(indicator, std::vector<TermPtr>{t}));
}

class MongologTests: public PrologTests<knowrob::MongologReasoner> {
protected:
	static std::string getPath(const std::string &filename)
	{ return std::filesystem::path("mongolog") / filename; }
};

TEST_F(MongologTests, arithmetic)	{ runTests(getPath("arithmetic.pl")); }
TEST_F(MongologTests, atoms)		{ runTests(getPath("atoms.pl")); }
TEST_F(MongologTests, comparison)	{ runTests(getPath("comparison.pl")); }
TEST_F(MongologTests, control)		{ runTests(getPath("control.pl")); }
TEST_F(MongologTests, database)		{ runTests(getPath("database.pl")); }
TEST_F(MongologTests, findall)		{ runTests(getPath("findall.pl")); }
TEST_F(MongologTests, fluents)		{ runTests(getPath("fluents.pl")); }
TEST_F(MongologTests, lists)		{ runTests(getPath("lists.pl")); }
TEST_F(MongologTests, meta)			{ runTests(getPath("meta.pl")); }
TEST_F(MongologTests, sgml)			{ runTests(getPath("sgml.pl")); }
TEST_F(MongologTests, terms)		{ runTests(getPath("terms.pl")); }
TEST_F(MongologTests, typecheck)	{ runTests(getPath("typecheck.pl")); }
TEST_F(MongologTests, unification)	{ runTests(getPath("unification.pl")); }
TEST_F(MongologTests, rdfs)			{ runTests(getPath("rdfs.pl")); }
TEST_F(MongologTests, annotation)	{ runTests(getPath("annotation.pl")); }
TEST_F(MongologTests, triple)		{ runTests(getPath("triple.plt")); }
TEST_F(MongologTests, owl)			{ runTests(getPath("owl.plt")); }
TEST_F(MongologTests, holds)		{ runTests(getPath("holds.pl")); }
TEST_F(MongologTests, temporal)		{ runTests(getPath("temporal.pl")); }
