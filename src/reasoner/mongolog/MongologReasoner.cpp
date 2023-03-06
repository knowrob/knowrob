/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>

#include "knowrob/Logger.h"
#include "knowrob/reasoner/prolog/PrologTests.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/reasoner/mongolog/MongologReasoner.h"

using namespace knowrob;

// make reasoner type accessible
KNOWROB_BUILTIN_REASONER("Mongolog", MongologReasoner)

MongologReasoner::MongologReasoner(const std::string &reasonerID)
: PrologReasoner(reasonerID)
{}

MongologReasoner::~MongologReasoner()
= default;

unsigned long MongologReasoner::getCapabilities() const
{
    return CAPABILITY_CONJUNCTIVE_QUERIES |
           CAPABILITY_DISJUNCTIVE_QUERIES |
           CAPABILITY_IMPORT_EXPORT |
           CAPABILITY_DYNAMIC_ASSERTIONS;
}

bool MongologReasoner::initializeDefaultPackages()
{
	static bool initialized = false;

	if(!initialized) {
		initialized = true;
		// load mongolog code once globally into the Prolog engine
		consult(std::filesystem::path("reasoner") / "mongolog" / "__init__.pl",
                "user", false);
	}

	// mongolog uses a special collection "one" that contains one empty document.
	// auto-create it if possible.
	eval(std::make_shared<Predicate>(Predicate("initialize_one_db",{})),
         nullptr, false);
	// initialize hierarchical organization of triple graphs
	eval(std::make_shared<Predicate>(Predicate("load_graph_structure",{})),
         nullptr, false);
	//
	eval(std::make_shared<Predicate>(Predicate("auto_drop_graphs",{})),
         nullptr, false);
    eval(std::make_shared<Predicate>(Predicate("update_rdf_predicates",{})),
         nullptr, false);

	// load RDFS and OWL model into DB, other RDF-XML files can be listed in settings.
	// TODO: only do below if mongolog reasoner includes semweb predicates.
	eval(std::make_shared<Predicate>(Predicate("setup_triple_collection",{})),
         nullptr, false);
    loadDataSource(std::make_shared<DataSource>(DataSource::RDF_XML_FORMAT, "owl/rdf-schema.xml"));
    loadDataSource(std::make_shared<DataSource>(DataSource::RDF_XML_FORMAT, "owl/owl.rdf"));

	return true;
}

const functor_t& MongologReasoner::callFunctor()
{
	static const auto call_f = PL_new_functor(PL_new_atom("mongolog_call"), 2);
	return call_f;
}

bool MongologReasoner::projectIntoEDB(const Statement &statement)
{
    // FIXME: handle time interval and confidence value!
    auto mongolog_project = std::make_shared<Predicate>(Predicate("mongolog_project",{
            statement.predicate()
    }));
    return eval(mongolog_project, nullptr, false);
}

bool MongologReasoner::importData(const std::filesystem::path &path)
{
    auto mng_dump = std::make_shared<Predicate>(Predicate("mongolog_import",{
            std::make_shared<StringTerm>(path.u8string())
    }));
    return eval(mng_dump, nullptr, false);
}

bool MongologReasoner::exportData(const std::filesystem::path &path)
{
    auto mng_dump = std::make_shared<Predicate>(Predicate("mongolog_export",{
        std::make_shared<StringTerm>(path.u8string())
    }));
    return eval(mng_dump, nullptr, false);
}

class MongologTests: public PrologTests<knowrob::MongologReasoner> {
protected:
	static void SetUpTestSuite() {
		reasoner()->load_rdf_xml("http://www.ease-crc.org/ont/SOMA.owl");
	}
	static std::string getPath(const std::string &filename)
	{ return std::filesystem::path("reasoner") / "mongolog" / filename; }
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