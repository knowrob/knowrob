/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include "knowrob/backend/mongo/MongoKnowledgeGraph.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/sources/OntologyParser.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/backend/redland/RedlandModel.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/reification/ReifiedTriple.h"
#include "knowrob/reasoner/prolog/PrologEngine.h"
#include "knowrob/backend/BackendInterface.h"
#include "knowrob/reasoner/prolog/PrologBackend.h"
#include "knowrob/reasoner/prolog/PrologReasoner.h"

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
	return kg;
}

// fixture class for testing
template <class BackendType>
class DataBackendTest : public ::testing::Test {
public:
	static std::shared_ptr<BackendType> kg_;
	static std::shared_ptr<QueryableBackend> queryable_;
	static std::shared_ptr<BackendInterface> backend_;

	static void SetUpTestSuite() {
		auto vocabulary = std::make_shared<Vocabulary>();
		auto backendManager = std::make_shared<BackendManager>(vocabulary);
		backend_ = std::make_shared<BackendInterface>(backendManager);
		kg_ = createBackend<BackendType>();
		backendManager->addPlugin("test", kg_);
		queryable_ = kg_;
		PrefixRegistry::registerPrefix("swrl_test", "http://knowrob.org/kb/swrl_test#");
	}

	// void TearDown() override {}
	template<class T>
	std::list<BindingsPtr> lookup(const T &data) {
		std::list<BindingsPtr> out;
		auto pattern = std::make_shared<FramedTriplePattern>(data);
		auto pattern_q = std::make_shared<GraphPathQuery>(pattern);
		// only queries that go through submitQuery are auto-expanded, so we
		// do the expansion here manually.
		auto expanded_q = queryable_->expand(pattern_q);
		backend_->query(queryable_, expanded_q->expanded, [&out](const BindingsPtr &next) {
			out.push_back(next);
		});
		return out;
	}

	static bool insertOne(const FramedTriple &triple) {
		return backend_->createTransaction(queryable_,BackendInterface::Insert)->commit(triple);
	}

	static bool mergeOne(const FramedTriple &triple) {
		return backend_->mergeInsert(queryable_, triple);
	}

	bool loadOntology(std::string_view path) {
		auto resolved = URI::resolve(path);
		auto origin = DataSource::getNameFromURI(resolved);
		auto vocab = backend_->vocabulary();
		OntologyParser parser(resolved, semweb::TripleFormat::RDF_XML);
		parser.setOrigin(ImportHierarchy::ORIGIN_USER);
		// filter is called for each triple, if it returns false, the triple is skipped
		parser.setFilter([vocab](const FramedTriple &triple) {
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

template <typename T> std::shared_ptr<T> DataBackendTest<T>::kg_;
template <typename T> std::shared_ptr<QueryableBackend> DataBackendTest<T>::queryable_;
template <typename T> std::shared_ptr<BackendInterface> DataBackendTest<T>::backend_;

static FramedTriplePattern parse(const std::string &str) {
	auto p = QueryParser::parsePredicate(str);
	return {p->arguments()[0], p->arguments()[1], p->arguments()[2], false};
}

using TestableBackends = ::testing::Types<RedlandModel, PrologBackend, MongoKnowledgeGraph>;
TYPED_TEST_SUITE(DataBackendTest, TestableBackends);

#define TEST_LOOKUP(x) DataBackendTest<TypeParam>::lookup(x)
#define TEST_INSERT_ONE(x) DataBackendTest<TypeParam>::insertOne(x)
#define TEST_MERGE_ONE(x) DataBackendTest<TypeParam>::mergeOne(x)

#define swrl_test_ "http://knowrob.org/kb/swrl_test#"

TYPED_TEST(DataBackendTest, Assert_a_b_c) {
	FramedTripleCopy data_abc(swrl_test_"a", swrl_test_"b", swrl_test_"c");
	EXPECT_NO_THROW(TEST_INSERT_ONE(data_abc));
	EXPECT_EQ(TEST_LOOKUP(data_abc).size(), 1);
	EXPECT_EQ(TEST_LOOKUP(FramedTripleCopy(swrl_test_"x",swrl_test_"b",swrl_test_"c")).size(), 0);
	EXPECT_EQ(TEST_LOOKUP(FramedTripleCopy(swrl_test_"a",swrl_test_"x",swrl_test_"c")).size(), 0);
	EXPECT_EQ(TEST_LOOKUP(FramedTripleCopy(swrl_test_"a",swrl_test_"b",swrl_test_"x")).size(), 0);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(A, swrl_test:b, swrl_test:c)")).size(), 1);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(A, swrl_test:x, swrl_test:c)")).size(), 0);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(swrl_test:a, B, swrl_test:c)")).size(), 1);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(swrl_test:x, B, swrl_test:c)")).size(), 0);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(swrl_test:a, swrl_test:b, C)")).size(), 1);
	EXPECT_EQ(TEST_LOOKUP(parse("triple(swrl_test:x, swrl_test:b, C)")).size(), 0);
}

TYPED_TEST(DataBackendTest, LoadSOMAandDUL) {
	EXPECT_NO_THROW(DataBackendTest<TypeParam>::loadOntology("owl/test/swrl.owl"));
	EXPECT_NO_THROW(DataBackendTest<TypeParam>::loadOntology("owl/test/datatype_test.owl"));
}

TYPED_TEST(DataBackendTest, QuerySubclassOf) {
	FramedTripleCopy triple(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	EXPECT_EQ(TEST_LOOKUP(triple).size(), 1);
}

TYPED_TEST(DataBackendTest, DeleteSubclassOf) {
	FramedTripleCopy triple(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	EXPECT_NO_THROW(DataBackendTest<TypeParam>::queryable_->removeOne(triple));
	EXPECT_EQ(TEST_LOOKUP(triple).size(), 0);
}

TYPED_TEST(DataBackendTest, AssertSubclassOf) {
	FramedTripleCopy existing(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	FramedTripleCopy not_existing(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"Car");
	EXPECT_NO_THROW(TEST_INSERT_ONE(existing));
	EXPECT_EQ(TEST_LOOKUP(existing).size(), 1);
	EXPECT_EQ(TEST_LOOKUP(not_existing).size(), 0);
}

TYPED_TEST(DataBackendTest, QueryNegatedTriple) {
	auto negated = std::make_shared<FramedTriplePattern>(
			QueryParser::parsePredicate("triple(swrl_test:x, swrl_test:p, swrl_test:y)"), true);
	EXPECT_EQ(TEST_LOOKUP(*negated).size(), 1);
	FramedTripleCopy statement(swrl_test_"x", swrl_test_"p", swrl_test_"y");
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(*negated).size(), 0);
}

TYPED_TEST(DataBackendTest, Knowledge) {
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "Lea", knowrob::XSDType::STRING);
	statement.setIsUncertain(false);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	statement.setIsUncertain(true);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
}

TYPED_TEST(DataBackendTest, KnowledgeOfAgent) {
	// assert knowledge of a named agent
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "Lea", knowrob::XSDType::STRING);
	statement.setIsUncertain(false);
	statement.setPerspective(swrl_test_"Lea");
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	// the statement is not known to be true for other agents
	statement.setPerspective(swrl_test_"Fred");
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
}

TYPED_TEST(DataBackendTest, Belief) {
	// assert uncertain statement
	FramedTripleCopy statement(swrl_test_"Fred", swrl_test_"hasName", "Fred", knowrob::XSDType::STRING);
	statement.setIsUncertain(true);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	// statement is filtered if knowledge operator is selected
	statement.setIsUncertain(false);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
}

TYPED_TEST(DataBackendTest, WithConfidence) {
	// assert uncertain statement with confidence=0.5
	FramedTripleCopy statement(swrl_test_"Bob", swrl_test_"hasName", "Bob", knowrob::XSDType::STRING);
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

TYPED_TEST(DataBackendTest, WithTimeInterval) {
	// assert a statement with time interval [5,10]
	FramedTripleCopy statement(swrl_test_"Alice", swrl_test_"hasName", "Alice", knowrob::XSDType::STRING);
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

TYPED_TEST(DataBackendTest, ExtendsTimeInterval) {
	// assert a statement with time interval [10,20]
	FramedTripleCopy statement(swrl_test_"Alice", swrl_test_"hasName", "Alice", knowrob::XSDType::STRING);
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
