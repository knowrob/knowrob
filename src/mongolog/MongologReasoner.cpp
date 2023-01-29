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

	if(!initialized) {
		initialized = true;
		// load mongolog code once globally into the Prolog engine
		consult(std::filesystem::path("mongolog") / "__init__.pl", "user", false);
	}

	// mongolog uses a special collection "one" that contains one empty document.
	// auto-create it if possible.
	eval(std::make_shared<Predicate>(Predicate("initialize_one_db",{})), nullptr, false);
	// initialize hierarchical organization of triple graphs
	eval(std::make_shared<Predicate>(Predicate("load_graph_structure",{})), nullptr, false);
	//
	eval(std::make_shared<Predicate>(Predicate("auto_drop_graphs",{})), nullptr, false);

	// load RDFS and OWL model into DB, other RDF-XML files can be listed in settings.
	// TODO: only do below if mongolog reasoner includes semweb predicates.
	eval(std::make_shared<Predicate>(Predicate("setup_triple_collection",{})), nullptr, false);
	loadDataFile(std::make_shared<DataFile>("owl/rdf-schema.xml", "rdf-xml"));
	loadDataFile(std::make_shared<DataFile>("owl/owl.rdf", "rdf-xml"));

	return true;
}

const functor_t& MongologReasoner::callFunctor()
{
	static const auto call_f = PL_new_functor(PL_new_atom("mongolog_call"), 2);
	return call_f;
}

class MongologTests: public PrologTests<knowrob::MongologReasoner> {
protected:
	static void SetUpTestSuite() {
		reasoner()->load_rdf_xml("http://www.ease-crc.org/ont/SOMA.owl");
	}
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
TEST_F(MongologTests, annotation)	{ runTests(getPath("annotation.pl")); }
TEST_F(MongologTests, triple)		{ runTests(getPath("triple.plt")); }
TEST_F(MongologTests, semweb)		{ runTests(getPath("semweb.plt")); }
TEST_F(MongologTests, holds)		{ runTests(getPath("holds.pl")); }
TEST_F(MongologTests, temporal)		{ runTests(getPath("temporal.pl")); }
