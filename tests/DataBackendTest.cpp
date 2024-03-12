/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include "knowrob/db/mongo/MongoKnowledgeGraph.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/db/OntologyParser.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/db/RedlandModel.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/reification/ReifiedBackend.h"
#include "knowrob/reification/ReifiedTriple.h"

using namespace knowrob;
using namespace knowrob::mongo;
using namespace knowrob::semweb;

template<typename T> std::shared_ptr<T> createBackend();

template<> std::shared_ptr<MongoKnowledgeGraph> createBackend<MongoKnowledgeGraph>() {
	auto kg = std::make_shared<MongoKnowledgeGraph>();
	kg->setVocabulary(std::make_shared<semweb::Vocabulary>());
	kg->setImportHierarchy(std::make_shared<semweb::ImportHierarchy>());
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
	kg->setVocabulary(std::make_shared<semweb::Vocabulary>());
	kg->setImportHierarchy(std::make_shared<semweb::ImportHierarchy>());
	kg->setHashesStorage(RedlandHashType::MEMORY);
	kg->initializeBackend();
	return kg;
}

// fixture class for testing
template <class BackendType>
class DataBackendTest : public ::testing::Test {
public:
	static std::shared_ptr<BackendType> kg_;
	static std::shared_ptr<QueryableBackend> queryable_;
	static std::shared_ptr<semweb::Vocabulary> vocabulary_;

	static void SetUpTestSuite() {
		vocabulary_ = std::make_shared<semweb::Vocabulary>();
		kg_ = createBackend<BackendType>();
		if (kg_->canStoreTripleContext()) {
			queryable_ = kg_;
		} else {
			queryable_ = std::make_shared<ReifiedBackend>(kg_);
			queryable_->setVocabulary(kg_->vocabulary());
			queryable_->setImportHierarchy(kg_->importHierarchy());
		}
		semweb::PrefixRegistry::registerPrefix("swrl_test", "http://knowrob.org/kb/swrl_test#");
	}

	// void TearDown() override {}
	template<class T>
	std::list<BindingsPtr> lookup(const T &data) {
		std::list<BindingsPtr> out;
		auto pattern = std::make_shared<FramedTriplePattern>(data);
		auto pattern_q = std::make_shared<GraphPathQuery>(pattern);
		// only queries that go through submitQuery are auto-expanded, so we
		// do the expansion here manually.
		auto expanded_q = QueryableBackend::expand(pattern_q);
		queryable_->query(expanded_q, [&out](const BindingsPtr &next) {
			out.push_back(next);
		});
		return out;
	}

	static bool insertOne(const FramedTriple &triple) {
		// FIXME: redundant with KnowledgeBase
		if (!kg_->canStoreTripleContext() && ReifiedTriple::isReifiable(triple)) {
			ReifiedTriple reification(triple, vocabulary_);
			bool allSuccess = true;
			for (auto &reified : reification) {
				allSuccess = kg_->insertOne(*reified.ptr) && allSuccess;
			}
			return allSuccess;
		} else {
			return kg_->insertOne(triple);
		}
	}

	bool loadOntology(std::string_view path) {
		auto resolved = URI::resolve(path);
		auto origin = DataSource::getNameFromURI(resolved);
		auto vocab = vocabulary_;
		OntologyParser parser(resolved, semweb::TripleFormat::RDF_XML, 100);
		// filter is called for each triple, if it returns false, the triple is skipped
		parser.setFilter([vocab](const FramedTriple &triple) {
			return !vocab->isAnnotationProperty(triple.predicate());
		});
		// define a prefix for naming blank nodes
		parser.setBlankPrefix(std::string("_") + origin);
		auto result = parser.run([this](const semweb::TripleContainerPtr &tripleContainer) {
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
template <typename T> std::shared_ptr<semweb::Vocabulary> DataBackendTest<T>::vocabulary_;

static FramedTriplePattern parse(const std::string &str) {
	auto p = QueryParser::parsePredicate(str);
	return {p->arguments()[0], p->arguments()[1], p->arguments()[2], false};
}

using TestableBackends = ::testing::Types<RedlandModel, MongoKnowledgeGraph>;
TYPED_TEST_SUITE(DataBackendTest, TestableBackends);

#define TEST_LOOKUP(x) DataBackendTest<TypeParam>::lookup(x)
#define TEST_INSERT_ONE(x) DataBackendTest<TypeParam>::insertOne(x)

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
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "Lea's name");
	statement.setStringValue("Lea's name");
	statement.setIsUncertain(false);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
	statement.setIsUncertain(true);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 1);
}

TYPED_TEST(DataBackendTest, KnowledgeOfAgent) {
	// assert knowledge of a named agent
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "Lea's name");
	statement.setStringValue("Lea's name");
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
	FramedTripleCopy statement(swrl_test_"Fred", swrl_test_"hasName", "Fred");
	statement.setStringValue("Fred");
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
	FramedTripleCopy statement(swrl_test_"Bob", swrl_test_"hasName", "Bob");
	statement.setStringValue("Bob");
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
	FramedTripleCopy statement(swrl_test_"Alice", swrl_test_"hasName", "Alice");
	statement.setStringValue("Alice");
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
	FramedTripleCopy statement(swrl_test_"Alice", swrl_test_"hasName", "Alice");
	statement.setStringValue("Alice");
	statement.setBegin(10.0);
	statement.setEnd(20.0);
	EXPECT_EQ(TEST_LOOKUP(statement).size(), 0);
	EXPECT_NO_THROW(TEST_INSERT_ONE(statement));
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
