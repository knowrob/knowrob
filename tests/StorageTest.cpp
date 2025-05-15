/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include "knowrob/storage/mongo/MongoKnowledgeGraph.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/semweb/OntologyParser.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/storage/redland/RedlandModel.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/storage/ReifiedTriple.h"
#include "knowrob/integration/prolog/PrologEngine.h"
#include "knowrob/storage/StorageInterface.h"
#include "knowrob/integration/prolog/PrologBackend.h"

using namespace knowrob;
using namespace knowrob::mongo;
using namespace knowrob::semweb;

template<typename T> std::shared_ptr<T> createBackend();

template<> std::shared_ptr<MongoKnowledgeGraph> createBackend<MongoKnowledgeGraph>() {
	auto kg = std::make_shared<MongoKnowledgeGraph>();
	kg->setVocabulary(std::make_shared<Vocabulary>());
	kg->initializeBackend(
			MongoKnowledgeGraph::DB_URI_DEFAULT,
			MongoKnowledgeGraph::DB_NAME_KNOWROB,
			MongoKnowledgeGraph::COLL_NAME_TESTS);
	kg->drop();
	kg->tripleCollection()->createTripleIndex();
	return kg;
}

template<> std::shared_ptr<RedlandModel> createBackend<RedlandModel>() {
	auto kg = std::make_shared<RedlandModel>();
	kg->setVocabulary(std::make_shared<Vocabulary>());
	kg->setHashesStorage(RedlandHashType::MEMORY);
	kg->initializeBackend();
	return kg;
}

template<> std::shared_ptr<PrologBackend> createBackend<PrologBackend>() {
	auto kg = std::make_shared<PrologBackend>();
	kg->setVocabulary(std::make_shared<Vocabulary>());
	kg->initializeBackend();
	kg->removeAll();
	return kg;
}

// fixture class for testing
template <class BackendType>
class StorageTest : public ::testing::Test {
public:
	static std::shared_ptr<BackendType> kg_;
	static std::shared_ptr<QueryableStorage> queryable_;
	static std::shared_ptr<StorageInterface> backend_;

	static void SetUpTestSuite() {
		auto vocabulary = std::make_shared<Vocabulary>();
		auto backendManager = std::make_shared<StorageManager>(vocabulary);
		backend_ = std::make_shared<StorageInterface>(backendManager);
		kg_ = createBackend<BackendType>();
		backendManager->addPlugin("test", PluginLanguage::CPP, kg_);
		queryable_ = kg_;
		PrefixRegistry::registerPrefix("swrl_test", "http://knowrob.org/kb/swrl_test#");
	}

	static void TearDownTestSuite() {
		queryable_ = nullptr;
		backend_ = nullptr;
		kg_ = nullptr;
	}

	// void TearDown() override {}
	template<class T>
	std::vector<BindingsPtr> lookup(const T &data) {
		std::vector<BindingsPtr> out;
		auto pattern = std::make_shared<TriplePattern>(data);
		auto pattern_q = std::make_shared<GraphPathQuery>(pattern);
		// only queries that go through submitQuery are auto-expanded, so we
		// do the expansion here manually.
		auto expanded_q = queryable_->expand(pattern_q);
		backend_->query(queryable_, expanded_q->expanded, [&out](const BindingsPtr &next) {
			out.push_back(next);
		});
		return out;
	}

	std::vector<BindingsPtr> query(const std::shared_ptr<GraphQuery> &query) {
		std::vector<BindingsPtr> out;
		// only queries that go through submitQuery are auto-expanded, so we
		// do the expansion here manually.
		auto expanded_q = queryable_->expand(query);
		backend_->query(queryable_, expanded_q->expanded, [&out](const BindingsPtr &next) {
			out.push_back(next);
		});
		return out;
	}

	static bool insertOne(const Triple &triple) {
		return backend_->createTransaction(queryable_, StorageInterface::Insert)->commit(triple);
	}

	static bool mergeOne(const Triple &triple) {
		return backend_->mergeInsert(queryable_, triple);
	}

	bool loadOntology(std::string_view path) {
		auto resolved = URI::resolve(path);
		auto origin = DataSource::getNameFromURI(resolved);
		auto vocab = backend_->vocabulary();
		OntologyParser parser(resolved, semweb::TripleFormat::RDF_XML);
		parser.setOrigin(ImportHierarchy::ORIGIN_USER);
		// filter is called for each triple, if it returns false, the triple is skipped
		parser.setFilter([vocab](const Triple &triple) {
			return !vocab->isAnnotationProperty(triple.predicate());
		});
		// define a prefix for naming blank nodes
		parser.setBlankPrefix(std::string("_") + origin);
		auto result = parser.run([](const TripleContainerPtr &tripleContainer) {
			queryable_->insertAll(tripleContainer);
		});
		if (result) {
			return true;
		} else {
			KB_WARN("Failed to parse ontology {} ({})", resolved, origin);
			return false;
		}
	}
};

template <typename T> std::shared_ptr<T> StorageTest<T>::kg_;
template <typename T> std::shared_ptr<QueryableStorage> StorageTest<T>::queryable_;
template <typename T> std::shared_ptr<StorageInterface> StorageTest<T>::backend_;

static TriplePattern parse(const std::string &str) {
	auto p = QueryParser::parsePredicate(str);
	return {p->arguments()[0], p->arguments()[1], p->arguments()[2], false};
}

using TestableBackends = ::testing::Types<RedlandModel, PrologBackend, MongoKnowledgeGraph>;
TYPED_TEST_SUITE(StorageTest, TestableBackends);

#define TEST_LOOKUP(x) StorageTest<TypeParam>::lookup(x)
#define TEST_QUERY(x) StorageTest<TypeParam>::query(x)
#define TEST_INSERT_ONE(x) StorageTest<TypeParam>::insertOne(x)
#define TEST_MERGE_ONE(x) StorageTest<TypeParam>::mergeOne(x)

#define swrl_test_ "http://knowrob.org/kb/swrl_test#"

TYPED_TEST(StorageTest, Assert_a_b_c) {
	TripleCopy data_abc(swrl_test_"a", swrl_test_"b", swrl_test_"c");
	EXPECT_NO_THROW(TEST_INSERT_ONE(data_abc));
	EXPECT_EQ(TEST_LOOKUP(data_abc).size(), 1);
	EXPECT_EQ(TEST_LOOKUP(TripleCopy(swrl_test_"x",swrl_test_"b",swrl_test_"c")).size(), 0);
	EXPECT_EQ(TEST_LOOKUP(TripleCopy(swrl_test_"a",swrl_test_"x",swrl_test_"c")).size(), 0);
	EXPECT_EQ(TEST_LOOKUP(TripleCopy(swrl_test_"a",swrl_test_"b",swrl_test_"x")).size(), 0);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(A, swrl_test:b, swrl_test:c)")).size(), 1);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(A, swrl_test:x, swrl_test:c)")).size(), 0);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(swrl_test:a, B, swrl_test:c)")).size(), 1);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(swrl_test:x, B, swrl_test:c)")).size(), 0);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(swrl_test:a, swrl_test:b, C)")).size(), 1);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(swrl_test:x, swrl_test:b, C)")).size(), 0);
}

TYPED_TEST(StorageTest, Assert_relative_iri) {
    TripleCopy data_abx("http://ont.de#a", "marriedTo", "http://ont.de#frank");
    EXPECT_NO_THROW(TEST_INSERT_ONE(data_abx));
    EXPECT_EQ(TEST_LOOKUP(data_abx).size(), 1);
    EXPECT_EQ(TEST_LOOKUP(parse("triple(X, marriedTo, 'http://ont.de#frank')")).size(), 1);
}

TYPED_TEST(StorageTest, QueryDuplicateVars) {
	EXPECT_EQ(TEST_LOOKUP(parse("triple(?x, swrl_test:b, ?x)")).size(), 0);
	TripleCopy data_aba(swrl_test_"a", swrl_test_"b", swrl_test_"a");
	EXPECT_NO_THROW(TEST_INSERT_ONE(data_aba));
	EXPECT_EQ(TEST_LOOKUP(parse("triple(?x, swrl_test:b, ?x)")).size(), 1);
}

TYPED_TEST(StorageTest, TripleWithOrigin) {
	TripleCopy data_cbd(swrl_test_"c", swrl_test_"b", swrl_test_"d");
	EXPECT_EQ(TEST_LOOKUP(data_cbd).size(), 0);
	data_cbd.setGraph(swrl_test_"origin1");
	EXPECT_NO_THROW(TEST_INSERT_ONE(data_cbd));
	EXPECT_EQ(TEST_LOOKUP(data_cbd).size(), 1);
	data_cbd.setGraph(swrl_test_"origin2");
	EXPECT_NO_THROW(TEST_INSERT_ONE(data_cbd));
	// lookup with given origin produces only one result
	EXPECT_EQ(TEST_LOOKUP(data_cbd).size(), 1);
	// but if origin is not specified, at least one result is expected
	// Note: some backends produce duplicate results, one for each origin.
	//       others may actually not provide the information about the origin at all,
	//       e.g in redland `librdf_model_find_statements` does not provide the
	//       origin information (context nodes in redland).
	EXPECT_GE(TEST_LOOKUP(parse("triple(swrl_test:c, swrl_test:b, swrl_test:d)")).size(), 1);
	// after removing the statement from one origin, the other one is still present
	EXPECT_NO_THROW(StorageTest<TypeParam>::queryable_->removeOne(data_cbd));
	EXPECT_EQ(TEST_LOOKUP(parse("triple(swrl_test:c, swrl_test:b, swrl_test:d)")).size(), 1);
	// removing again with origin2 should not change anything
	EXPECT_NO_THROW(StorageTest<TypeParam>::queryable_->removeOne(data_cbd));
	EXPECT_EQ(TEST_LOOKUP(parse("triple(swrl_test:c, swrl_test:b, swrl_test:d)")).size(), 1);
	// but when retracting the triple with origin1, it is gone
	data_cbd.setGraph(swrl_test_"origin1");
	EXPECT_NO_THROW(StorageTest<TypeParam>::queryable_->removeOne(data_cbd));
	EXPECT_EQ(TEST_LOOKUP(parse("triple(swrl_test:c, swrl_test:b, swrl_test:d)")).size(), 0);
}

TYPED_TEST(StorageTest, LoadTestOntology) {
	EXPECT_NO_THROW(StorageTest<TypeParam>::loadOntology("tests/owl/swrl.owl"));
	EXPECT_NO_THROW(StorageTest<TypeParam>::loadOntology("tests/owl/datatype_test.owl"));
}

TYPED_TEST(StorageTest, QuerySubclassOf) {
	TripleCopy triple(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	EXPECT_EQ(TEST_LOOKUP(triple).size(), 1);
}

TYPED_TEST(StorageTest, QueryRange) {
	TripleCopy triple(
			swrl_test_"hasParent",
			rdfs::range->stringForm().data(),
			swrl_test_"TestThing");
	EXPECT_EQ(TEST_LOOKUP(triple).size(), 1);
}

TYPED_TEST(StorageTest, DeleteSubclassOf) {
	TripleCopy triple(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	EXPECT_NO_THROW(StorageTest<TypeParam>::queryable_->removeOne(triple));
	EXPECT_EQ(TEST_LOOKUP(triple).size(), 0);
}

TYPED_TEST(StorageTest, AssertSubclassOf) {
	TripleCopy existing(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	TripleCopy not_existing(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"Car");
	EXPECT_NO_THROW(TEST_INSERT_ONE(existing));
	EXPECT_EQ(TEST_LOOKUP(existing).size(), 1);
	EXPECT_EQ(TEST_LOOKUP(not_existing).size(), 0);
}

TYPED_TEST(StorageTest, PathQuery) {
	// hasAncestor(Lea, ?x) & hasAncestor(?x, Rex) -> ?x = Fred
	auto q1 = std::make_shared<TriplePattern>(
			QueryParser::parsePredicate("triple(swrl_test:'Lea', swrl_test:hasAncestor, ?x)"));
	auto q2 = std::make_shared<TriplePattern>(
			QueryParser::parsePredicate("triple(?x, swrl_test:hasAncestor, swrl_test:'Rex')"));
	auto query = std::make_shared<GraphPathQuery>(std::vector<TriplePatternPtr>{q1, q2});
	auto fred_iri = IRIAtom::Tabled(swrl_test_"Fred");
	auto fred_var = std::make_shared<Variable>("x");
	auto results = TEST_QUERY(query);
	// Note: there are two assertions satisfying q1:
	//    (1) Lea -hasAncestor-> Fred, and (2) Lea -hasParent-> Fred
	// It is ok though if backends produce duplicate results or not. so we check here for at least 1 result.
	EXPECT_GE(results.size(), 1);
	for (auto &result : results) {
		EXPECT_EQ(*result, Bindings({{fred_var, fred_iri}}));
	}
}

TYPED_TEST(StorageTest, UnionQuery) {
	// hasAncestor(Fred, ?x) | hasSibling(Fred, ?x) -> ?x = Rex | Ernest
	auto q1 = std::make_shared<TriplePattern>(
			QueryParser::parsePredicate("triple(swrl_test:'Fred', swrl_test:hasAncestor, ?x)"));
	auto q2 = std::make_shared<TriplePattern>(
			QueryParser::parsePredicate("triple(swrl_test:'Fred', swrl_test:hasSibling, ?x)"));
	auto t1 = std::make_shared<GraphPattern>(q1);
	auto t2 = std::make_shared<GraphPattern>(q2);
	auto unionTerm = std::make_shared<GraphUnion>(
			std::vector<std::shared_ptr<GraphTerm>>{t1,t2});
	auto results = TEST_QUERY(std::make_shared<GraphQuery>(unionTerm));
	EXPECT_EQ(results.size(), 2);
}

TYPED_TEST(StorageTest, QueryNegatedTriple) {
	auto negated = std::make_shared<TriplePattern>(
			QueryParser::parsePredicate("triple(swrl_test:x, swrl_test:p, swrl_test:y)"), true);
	EXPECT_EQ(TEST_LOOKUP(*negated).size(), 1);
	TripleCopy statement(swrl_test_"x", swrl_test_"p", swrl_test_"y");
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(*negated).size(), 0);
}

TYPED_TEST(StorageTest, Knowledge) {
	TripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "Lea", knowrob::XSDType::STRING);
	statement.setIsUncertain(false);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	statement.setIsUncertain(true);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
}

TYPED_TEST(StorageTest, KnowledgeOfAgent) {
	// assert knowledge of a named agent
	TripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "Lea", knowrob::XSDType::STRING);
	statement.setIsUncertain(false);
	statement.setPerspective(swrl_test_"Lea");
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	// the statement is not known to be true for other agents
	statement.setPerspective(swrl_test_"Fred");
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
}

TYPED_TEST(StorageTest, Belief) {
	// assert uncertain statement
	TripleCopy statement(swrl_test_"Fred", swrl_test_"hasName", "Fred", knowrob::XSDType::STRING);
	statement.setIsUncertain(true);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	// statement is filtered if knowledge operator is selected
	statement.setIsUncertain(false);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
}

TYPED_TEST(StorageTest, WithConfidence) {
	// assert uncertain statement with confidence=0.5
	TripleCopy statement(swrl_test_"Bob", swrl_test_"hasName", "Bob", knowrob::XSDType::STRING);
	statement.setIsUncertain(true);
	statement.setConfidence(0.5);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	// confidence threshold of 0.0 does not filter the statement
	statement.setConfidence(0.0);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	// confidence threshold of 0.9 filters the statement
	statement.setConfidence(0.9);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
}

TYPED_TEST(StorageTest, WithTimeInterval) {
	// assert a statement with time interval [5,10]
	TripleCopy statement(swrl_test_"Alice", swrl_test_"hasName", "Alice", knowrob::XSDType::STRING);
	statement.setBegin(5.0);
	statement.setEnd(10.0);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	// no solution because statement only known to be true until 10.0
	statement.setEnd(20.0);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	// but temporal overlap is sufficient if "sometimes" operator is used
	statement.setIsOccasional(true);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
}

TYPED_TEST(StorageTest, ExtendsTimeInterval) {
	// assert a statement with time interval [10,20]
	TripleCopy statement(swrl_test_"Alice", swrl_test_"hasName", "Alice", knowrob::XSDType::STRING);
	statement.setBegin(10.0);
	statement.setEnd(20.0);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	EXPECT_NO_THROW(TEST_MERGE_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	// time interval was merged with existing one into [5,20]
	statement.setBegin(5.0);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	// no solution because statement only known to be true since 5.0
	statement.setBegin(0.0);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	// temporal overlap is sufficient if "sometimes" operator is used
	statement.setIsOccasional(true);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
}
