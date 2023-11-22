#include <gtest/gtest.h>

#include "KnowledgeBaseTest.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryError.h"

using namespace knowrob;

#define KB_TEST_SETTINGS_FILE "tests/settings/kb-test.json"

// TODO:
// here main interface of KnoweldgeBase should be tested, in particular:
// - submitQuery*
// - insert*
//
// what still is needed:
// - use some dummy reasoner with fixed input-output mapping that can be tested easily
//

std::shared_ptr<knowrob::KnowledgeBase> KnowledgeBaseTest::kb_;

void KnowledgeBaseTest::SetUpTestSuite() {
	// initialize a KB, setup database backend for testing, insert tmp data on which queries can be evaluated
	kb_ = std::make_shared<KnowledgeBase>(KB_TEST_SETTINGS_FILE);
}

static std::vector<SubstitutionPtr> lookupAll(const std::string &queryString) {
	auto answerStream = KnowledgeBaseTest::kb_->submitQuery(
			QueryParser::parse(queryString), QUERY_FLAG_ALL_SOLUTIONS);
	auto answerQueue = answerStream->createQueue();
	std::vector<SubstitutionPtr> out;
	while(true) {
		auto solution = answerQueue->pop_front();
		if(AnswerStream::isEOS(solution)) break;
		out.push_back(solution->substitution());
	}
	return out;
}

static bool containsAnswer(const std::vector<SubstitutionPtr> &answers, const std::string &key, const TermPtr &value) {
	Variable v_key(key);
	for(auto x: answers) {
		if(x->contains(v_key)) {
			auto actual = x->get(v_key);
			if(*value == *actual) {
				return true;
			}
		}
	}
	return false;
}

TEST_F(KnowledgeBaseTest, undefinedNamespace) {
	EXPECT_THROW(lookupAll("undefined:hasSibling(swrl_test:Fred, X)"), QueryError);
}

TEST_F(KnowledgeBaseTest, atomic_EDB) {
	EXPECT_EQ(lookupAll("swrl_test:hasSibling(swrl_test:Fred, X)").size(), 1);
	EXPECT_EQ(*lookupAll("swrl_test:hasSibling(swrl_test:Fred, X)")[0]->get("X"),
	          *QueryParser::parseConstant("swrl_test:Ernest"));
	EXPECT_EQ(lookupAll("swrl_test:hasSibling(swrl_test:Lea, X)").size(), 0);
	EXPECT_EQ(lookupAll("swrl_test:hasSibling(X, swrl_test:Ernest)").size(), 1);
	EXPECT_EQ(lookupAll("swrl_test:hasSibling(swrl_test:Fred, swrl_test:Ernest)").size(), 1);
}

TEST_F(KnowledgeBaseTest, conjunctive_EDB) {
	const auto queryString = "swrl_test:hasSibling(swrl_test:Fred,X) , swrl_test:hasNumber(X,Num)";
	EXPECT_EQ(lookupAll(queryString).size(), 1);
	EXPECT_EQ(*lookupAll(queryString)[0]->get("Num"), StringTerm("123456"));
}

TEST_F(KnowledgeBaseTest, disjunctive_EDB) {
	const auto queryString = "swrl_test:hasSibling(swrl_test:Fred,X) ; swrl_test:hasAncestor(swrl_test:Fred,X)";
	EXPECT_EQ(lookupAll(queryString).size(), 2);
	EXPECT_TRUE(containsAnswer(lookupAll(queryString), "X", QueryParser::parseConstant("swrl_test:Ernest")));
	EXPECT_TRUE(containsAnswer(lookupAll(queryString), "X", QueryParser::parseConstant("swrl_test:Rex")));
}

TEST_F(KnowledgeBaseTest, negated_EDB) {
	const auto queryString = "~swrl_test:hasSibling(swrl_test:Lea,X)";
	EXPECT_EQ(lookupAll(queryString).size(), 1);
	EXPECT_TRUE(lookupAll(queryString)[0]->empty());
}

TEST_F(KnowledgeBaseTest, complex_EDB) {
	const auto queryString = "(swrl_test:hasSibling(swrl_test:Fred,X) ; "\
							 "swrl_test:hasAncestor(swrl_test:Fred,X)) , swrl_test:hasSibling(X,Y)";
	EXPECT_EQ(lookupAll(queryString).size(), 1);
	EXPECT_TRUE(containsAnswer(lookupAll(queryString), "X", QueryParser::parseConstant("swrl_test:Ernest")));
}
