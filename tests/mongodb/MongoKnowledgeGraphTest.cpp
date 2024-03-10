/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include "knowrob/db/mongo/MongoKnowledgeGraph.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/db/OntologyParser.h"
#include "knowrob/semweb/rdfs.h"

using namespace knowrob;
using namespace knowrob::mongo;
using namespace knowrob::semweb;

// fixture class for testing
class MongoKnowledgeGraphTest : public ::testing::Test {
protected:
	static std::shared_ptr<MongoKnowledgeGraph> kg_;
	static std::shared_ptr<semweb::Vocabulary> vocabulary_;

	static void SetUpTestSuite() {
		vocabulary_ = std::make_shared<semweb::Vocabulary>();

		kg_ = std::make_shared<MongoKnowledgeGraph>();
		kg_->setVocabulary(vocabulary_);
		kg_->setImportHierarchy(std::make_shared<semweb::ImportHierarchy>());
		kg_->initializeBackend(
				MongoKnowledgeGraph::DB_URI_DEFAULT,
				MongoKnowledgeGraph::DB_NAME_KNOWROB,
				MongoKnowledgeGraph::COLL_NAME_TESTS);
		kg_->drop();
		kg_->tripleCollection()->createTripleIndex();
	}

	// void TearDown() override {}
	template<class T>
	std::list<BindingsPtr> lookup(const T &data) {
		auto cursor = kg_->lookup(FramedTriplePattern(data));
		std::list<BindingsPtr> out;
		while (true) {
			auto next = std::make_shared<Bindings>();
			if (cursor->nextBindings(next)) {
				out.push_back(next);
			} else {
				break;
			}
		}
		return out;
	}

	static FramedTriplePattern parse(const std::string &str) {
		auto p = QueryParser::parsePredicate(str);
		return {p->arguments()[0], p->arguments()[1], p->arguments()[2], false};
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
			kg_->insertAll(tripleContainer);
		});
		if (result) {
			return true;
		} else {
			KB_WARN("Failed to parse ontology {} ({})", resolved, origin);
			return false;
		}
	}
};

std::shared_ptr<MongoKnowledgeGraph> MongoKnowledgeGraphTest::kg_ = {};
std::shared_ptr<semweb::Vocabulary> MongoKnowledgeGraphTest::vocabulary_ = {};

TEST_F(MongoKnowledgeGraphTest, Assert_a_b_c) {
	FramedTripleCopy data_abc("a", "b", "c");
	EXPECT_NO_THROW(kg_->insertOne(data_abc));
	EXPECT_EQ(lookup(data_abc).size(), 1);
	EXPECT_EQ(lookup(parse("triple(x,b,c)")).size(), 0);
	EXPECT_EQ(lookup(parse("triple(a,x,c)")).size(), 0);
	EXPECT_EQ(lookup(parse("triple(a,b,x)")).size(), 0);
	EXPECT_EQ(lookup(parse("triple(A,b,c)")).size(), 1);
	EXPECT_EQ(lookup(parse("triple(A,x,c)")).size(), 0);
	EXPECT_EQ(lookup(parse("triple(a,B,c)")).size(), 1);
	EXPECT_EQ(lookup(parse("triple(x,B,c)")).size(), 0);
	EXPECT_EQ(lookup(parse("triple(a,b,C)")).size(), 1);
	EXPECT_EQ(lookup(parse("triple(x,b,C)")).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, LoadSOMAandDUL) {
	EXPECT_NO_THROW(loadOntology("owl/test/swrl.owl"));
	EXPECT_NO_THROW(loadOntology("owl/test/datatype_test.owl"));
}

#define swrl_test_ "http://knowrob.org/kb/swrl_test#"

TEST_F(MongoKnowledgeGraphTest, QueryTriple) {
	FramedTripleCopy triple(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	EXPECT_EQ(lookup(triple).size(), 1);
}

TEST_F(MongoKnowledgeGraphTest, QueryNegatedTriple) {
	auto negated = std::make_shared<FramedTriplePattern>(
			QueryParser::parsePredicate("p(x,y)"), true);
	EXPECT_EQ(lookup(*negated).size(), 1);
	FramedTripleCopy statement("x", "p", "y");
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(*negated).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, DeleteSubclassOf) {
	FramedTripleCopy triple(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	EXPECT_NO_THROW(kg_->removeOne(triple));
	EXPECT_EQ(lookup(triple).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, AssertSubclassOf) {
	FramedTripleCopy existing(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	FramedTripleCopy not_existing(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"Car");
	EXPECT_NO_THROW(kg_->insertOne(existing));
	EXPECT_EQ(lookup(existing).size(), 1);
	EXPECT_EQ(lookup(not_existing).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, Knowledge) {
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "X");
	statement.setIsUncertain(false);
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	statement.setIsUncertain(true);
	EXPECT_EQ(lookup(statement).size(), 1);
}

TEST_F(MongoKnowledgeGraphTest, KnowledgeOfAgent) {
	// assert knowledge of a named agent
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "Y");
	statement.setIsUncertain(false);
	statement.setPerspective("agent_a");
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	// the statement is not known to be true for other agents
	statement.setPerspective("agent_b");
	EXPECT_EQ(lookup(statement).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, Belief) {
	// assert uncertain statement
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "Lea");
	statement.setIsUncertain(true);
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	// statement is filtered if knowledge operator is selected
	statement.setIsUncertain(false);
	EXPECT_EQ(lookup(statement).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, WithConfidence) {
	// assert uncertain statement with confidence=0.5
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "A");
	statement.setIsUncertain(true);
	statement.setConfidence(0.5);
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	// confidence threshold of 0.0 does not filter the statement
	statement.setConfidence(0.0);
	EXPECT_EQ(lookup(statement).size(), 1);
	// confidence threshold of 0.9 filters the statement
	statement.setConfidence(0.9);
	EXPECT_EQ(lookup(statement).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, WithTimeInterval) {
	// assert a statement with time interval [5,10]
	FramedTripleCopy statement(swrl_test_"Rex", swrl_test_"hasName", "Rex");
	statement.setBegin(5.0);
	statement.setEnd(10.0);
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	// no solution because statement only known to be true until 10.0
	statement.setEnd(20.0);
	EXPECT_EQ(lookup(statement).size(), 0);
	// but temporal overlap is sufficient if "sometimes" operator is used
	statement.setIsOccasional(true);
	EXPECT_EQ(lookup(statement).size(), 1);
}

TEST_F(MongoKnowledgeGraphTest, ExtendsTimeInterval) {
	// assert a statement with time interval [10,20]
	FramedTripleCopy statement(swrl_test_"Rex", swrl_test_"hasName", "Rex");
	statement.setBegin(10.0);
	statement.setEnd(20.0);
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	// time interval was merged with existing one into [5,20]
	statement.setBegin(5.0);
	EXPECT_EQ(lookup(statement).size(), 1);
	// no solution because statement only known to be true since 5.0
	statement.setBegin(0.0);
	EXPECT_EQ(lookup(statement).size(), 0);
	// temporal overlap is sufficient if "sometimes" operator is used
	statement.setIsOccasional(true);
	EXPECT_EQ(lookup(statement).size(), 1);
}
