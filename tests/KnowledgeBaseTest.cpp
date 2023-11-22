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
