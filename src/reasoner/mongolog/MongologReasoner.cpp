/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include <boost/stacktrace.hpp>

#include "knowrob/Logger.h"
#include "knowrob/reasoner/prolog/PrologTests.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/reasoner/mongolog/MongologReasoner.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/URI.h"
#include "knowrob/reasoner/ReasonerError.h"

using namespace knowrob;

// make reasoner type accessible
KNOWROB_BUILTIN_REASONER("Mongolog", MongologReasoner)

// foreign predicates
foreign_t pl_is_readonly2(term_t, term_t);

foreign_t pl_db_name3(term_t, term_t, term_t);

foreign_t pl_uri3(term_t, term_t, term_t);

foreign_t mng_drop_graph3(term_t, term_t, term_t);

foreign_t pl_load_triples_cpp4(term_t, term_t, term_t, term_t);

foreign_t pl_rdf_current_property_cpp3(term_t t_reasonerManager, term_t t_reasonerModule, term_t t_propertyIRI);

foreign_t pl_assert_triple_cpp9(term_t, term_t, term_t, term_t, term_t, term_t, term_t, term_t, term_t);

MongologReasoner::MongologReasoner()
		: PrologReasoner() {}

MongologReasoner::~MongologReasoner()
= default;

bool MongologReasoner::initializeDefaultPackages() {
	static bool initialized = false;

	if (!initialized) {
		initialized = true;
		// load mongolog code once globally into the Prolog engine
		consult(std::filesystem::path("reasoner") / "mongolog" / "__init__.pl",
				"user", false);

		PL_register_foreign("mng_is_readonly_cpp",
							2, (pl_function_t) pl_is_readonly2, 0);
		PL_register_foreign("mng_db_name_cpp",
							3, (pl_function_t) pl_db_name3, 0);
		PL_register_foreign("mng_uri_cpp",
							3, (pl_function_t) pl_uri3, 0);

		PL_register_foreign("mng_drop_graph_cpp",
							3, (pl_function_t) mng_drop_graph3, 0);
		PL_register_foreign("mng_load_triples_cpp",
							4, (pl_function_t) pl_load_triples_cpp4, 0);
		PL_register_foreign("mng_rdf_current_property_cpp",
							3, (pl_function_t) pl_rdf_current_property_cpp3, 0);
		PL_register_foreign("mng_assert_triple_cpp",
							9, (pl_function_t) pl_assert_triple_cpp9, 0);
	}

	return true;
}

bool MongologReasoner::loadConfig(const ReasonerConfig &reasonerConfiguration) {
	if (!PrologReasoner::loadConfig(reasonerConfiguration)) return false;

	if (!knowledgeGraph_) {
		knowledgeGraph_ = std::make_shared<MongoKnowledgeGraph>("mongodb://localhost:27017", "knowrob", "triples");
		KB_WARN("Falling back to default configuration for MongoDB!");
	}
	importHierarchy_ = knowledgeGraph_->importHierarchy();
	importHierarchy_->addDirectImport("user", reasonerName());

	// load some common ontologies
	loadDataSource(std::make_shared<DataSource>(DataSource::RDF_XML_FORMAT, "owl/rdf-schema.xml"));
	loadDataSource(std::make_shared<DataSource>(DataSource::RDF_XML_FORMAT, "owl/owl.rdf"));

	return true;
}

void MongologReasoner::setDataBackend(const DataBackendPtr &backend) {
	knowledgeGraph_ = std::dynamic_pointer_cast<MongoKnowledgeGraph>(backend);
	if (!knowledgeGraph_) {
		throw ReasonerError(
				"Unexpected data knowledgeGraph used for Mongolog reasoner. MongoKnowledgeGraph must be used.",
				boost::stacktrace::stacktrace());
	}
}

const functor_t &MongologReasoner::callFunctor() {
	static const auto call_f = PL_new_functor(PL_new_atom("mongolog_call"), 2);
	return call_f;
}

/*
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
*/

static inline std::shared_ptr<MongologReasoner> getMongologReasoner(term_t t_reasonerManager,
																	term_t t_reasonerModule) {
	auto definedReasoner = PrologReasoner::getDefinedReasoner(t_reasonerManager, t_reasonerModule);
	if (!definedReasoner) {
		KB_ERROR("unable to find reasoner with id '{}' (manager id: {}).",
				 *PrologQuery::constructTerm(t_reasonerModule),
				 *PrologQuery::constructTerm(t_reasonerManager));
		return {};
	}
	auto reasoner = definedReasoner->reasoner();
	auto mongolog = std::dynamic_pointer_cast<MongologReasoner>(reasoner);
	if (!mongolog) {
		KB_ERROR("reasoner with id '{}' (manager id: {}) is not a mongolog reasoner.",
				 *PrologQuery::constructTerm(t_reasonerModule),
				 *PrologQuery::constructTerm(t_reasonerManager));
	}
	return mongolog;
}


foreign_t pl_is_readonly2(term_t t_reasonerManager, term_t t_reasonerModule) {
	auto mongolog = getMongologReasoner(t_reasonerManager, t_reasonerModule);
	char *graph;
	if (mongolog) {
		return mongolog->knowledgeGraph()->isReadOnly();
	}
	return false;
}

foreign_t pl_db_name3(term_t t_reasonerManager, term_t t_reasonerModule, term_t t_dbName) {
	auto mongolog = getMongologReasoner(t_reasonerManager, t_reasonerModule);
	if (mongolog) {
		auto &dbName = mongolog->knowledgeGraph()->dbName();
		return PL_unify_atom_chars(t_dbName, dbName.c_str());
	}
	return false;
}

foreign_t pl_uri3(term_t t_reasonerManager, term_t t_reasonerModule, term_t t_uri) {
	auto mongolog = getMongologReasoner(t_reasonerManager, t_reasonerModule);
	if (mongolog) {
		auto &uri = mongolog->knowledgeGraph()->dbURI();
		return PL_unify_atom_chars(t_uri, uri.c_str());
	}
	return false;
}


foreign_t pl_rdf_current_property_cpp3(term_t t_reasonerManager,
									   term_t t_reasonerModule,
									   term_t t_propertyIRI) {
	auto mongolog = getMongologReasoner(t_reasonerManager, t_reasonerModule);
	char *propertyIRI;
	if (mongolog && PL_get_atom_chars(t_propertyIRI, &propertyIRI)) {
		return mongolog->knowledgeGraph()->vocabulary()->isDefinedProperty(propertyIRI);
	}
	return false;
}

foreign_t pl_load_triples_cpp4(term_t t_reasonerManager,
							   term_t t_reasonerModule,
							   term_t t_ontologyURI,
							   term_t t_parentGraph) {
	auto mongolog = getMongologReasoner(t_reasonerManager, t_reasonerModule);
	char *ontologyURI;
	if (mongolog && PL_get_atom_chars(t_ontologyURI, &ontologyURI)) {
		auto &kg = mongolog->knowledgeGraph();

		if (!kg->loadFile(ontologyURI, TripleFormat::RDF_XML, *DefaultGraphSelector())) return false;

		char *parentGraph;
		if (!PL_is_variable(t_parentGraph) && PL_get_atom_chars(t_parentGraph, &parentGraph)) {
			// FIXME: redundant
			auto resolved = URI::resolve(ontologyURI);
			auto graphName = KnowledgeGraph::getNameFromURI(resolved);
			mongolog->importHierarchy()->addDirectImport(parentGraph, graphName);
		}
		return true;
	}
	return false;
}


foreign_t mng_drop_graph3(term_t t_reasonerManager,
						  term_t t_reasonerModule,
						  term_t t_graph) {
	auto mongolog = getMongologReasoner(t_reasonerManager, t_reasonerModule);
	char *graph;
	if (mongolog && PL_get_atom_chars(t_graph, &graph)) {
		mongolog->knowledgeGraph()->dropGraph(graph);
		return true;
	}
	return false;
}

foreign_t pl_assert_triple_cpp9(term_t t_reasonerManager,
								term_t t_reasonerModule,
								term_t t_subjectTerm,
								term_t t_propertyTerm,
								term_t t_objectTerm,
								term_t t_graphTerm,
								term_t t_beginTerm,
								term_t t_endTerm,
								term_t t_confidenceTerm) {
	auto mongolog = getMongologReasoner(t_reasonerManager, t_reasonerModule);
	if (mongolog) {
		StatementData tripleData;

		// "s" field
		auto subjectTerm = PrologQuery::constructTerm(t_subjectTerm);
		if (subjectTerm->type() != TermType::STRING) throw QueryError("invalid subject term {}", *subjectTerm);
		tripleData.subject = ((StringTerm *) subjectTerm.get())->value().c_str();

		// "p" field
		auto propertyTerm = PrologQuery::constructTerm(t_propertyTerm);
		if (propertyTerm->type() != TermType::STRING) throw QueryError("invalid property term {}", *propertyTerm);
		tripleData.predicate = ((StringTerm *) propertyTerm.get())->value().c_str();

		// "o" field
		auto objectTerm = PrologQuery::constructTerm(t_objectTerm);
		switch (objectTerm->type()) {
			case TermType::STRING:
				tripleData.objectType = RDF_STRING_LITERAL;
				tripleData.object = ((StringTerm *) objectTerm.get())->value().c_str();
				break;
			case TermType::DOUBLE:
				tripleData.objectType = RDF_DOUBLE_LITERAL;
				tripleData.objectDouble = ((DoubleTerm *) objectTerm.get())->value();
				break;
			case TermType::INT32:
			case TermType::LONG:
				tripleData.objectType = RDF_INT64_LITERAL;
				tripleData.objectInteger = ((LongTerm *) objectTerm.get())->value();
				break;
			default:
				throw QueryError("object term {} of triple has an invalid type", *objectTerm);
		}

		// "g" field
		TermPtr graphTerm;
		if (!PL_is_variable(t_graphTerm)) {
			graphTerm = PrologQuery::constructTerm(t_graphTerm);
			if (graphTerm->type() != TermType::STRING) throw QueryError("invalid property term {}", *graphTerm);
			tripleData.graph = ((StringTerm *) graphTerm.get())->value().c_str();
		} else {
			tripleData.graph = "user";
		}

		// "c" field
		TermPtr confidenceTerm;
		if (!PL_is_variable(t_confidenceTerm)) {
			confidenceTerm = PrologQuery::constructTerm(t_confidenceTerm);
			switch (confidenceTerm->type()) {
				case TermType::DOUBLE:
					tripleData.confidence = ((DoubleTerm *) confidenceTerm.get())->value();
					break;
				default:
					throw QueryError("invalid confidence term {}", *confidenceTerm);
			}
		}

		// "b" field
		TermPtr beginTerm;
		if (!PL_is_variable(t_beginTerm)) {
			beginTerm = PrologQuery::constructTerm(t_beginTerm);
			switch (beginTerm->type()) {
				case TermType::DOUBLE:
					tripleData.begin = ((DoubleTerm *) beginTerm.get())->value();
					break;
				case TermType::INT32:
					tripleData.begin = ((Integer32Term *) beginTerm.get())->value();
					break;
				case TermType::LONG:
					tripleData.begin = ((LongTerm *) beginTerm.get())->value();
					break;
				default:
					throw QueryError("invalid begin term {}", *beginTerm);
			}
		}

		// "e" field
		TermPtr endTerm;
		if (!PL_is_variable(t_endTerm)) {
			endTerm = PrologQuery::constructTerm(t_endTerm);
			switch (endTerm->type()) {
				case TermType::DOUBLE:
					tripleData.end = ((DoubleTerm *) endTerm.get())->value();
					break;
				case TermType::INT32:
					tripleData.end = ((Integer32Term *) endTerm.get())->value();
					break;
				case TermType::LONG:
					tripleData.end = ((LongTerm *) endTerm.get())->value();
					break;
				default:
					throw QueryError("invalid end term {}", *endTerm);
			}
		}

		mongolog->knowledgeGraph()->insertOne(tripleData);
		return true;
	}
	return false;
}


class MongologTests : public PrologTests<knowrob::MongologReasoner> {
protected:
	static void SetUpTestSuite() {
		reasoner()->load_rdf_xml("http://www.ease-crc.org/ont/SOMA.owl");
	}

	static std::string getPath(const std::string &filename) {
		return std::filesystem::path("reasoner") / "mongolog" / filename;
	}
};

TEST_F(MongologTests, arithmetic) { runTests(getPath("arithmetic.pl")); }

TEST_F(MongologTests, atoms) { runTests(getPath("atoms.pl")); }

TEST_F(MongologTests, comparison) { runTests(getPath("comparison.pl")); }

TEST_F(MongologTests, control) { runTests(getPath("control.pl")); }

TEST_F(MongologTests, database) { runTests(getPath("database.pl")); }

TEST_F(MongologTests, findall) { runTests(getPath("findall.pl")); }

TEST_F(MongologTests, fluents) { runTests(getPath("fluents.pl")); }

TEST_F(MongologTests, lists) { runTests(getPath("lists.pl")); }

TEST_F(MongologTests, meta) { runTests(getPath("meta.pl")); }

TEST_F(MongologTests, sgml) { runTests(getPath("sgml.pl")); }

TEST_F(MongologTests, terms) { runTests(getPath("terms.pl")); }

TEST_F(MongologTests, typecheck) { runTests(getPath("typecheck.pl")); }

TEST_F(MongologTests, unification) { runTests(getPath("unification.pl")); }

TEST_F(MongologTests, annotation) { runTests(getPath("annotation.pl")); }

TEST_F(MongologTests, triple) { runTests(getPath("triple.plt")); }

TEST_F(MongologTests, semweb) { runTests(getPath("semweb.plt")); }

TEST_F(MongologTests, holds) { runTests(getPath("holds.pl")); }

TEST_F(MongologTests, temporal) { runTests(getPath("temporal.pl")); }
