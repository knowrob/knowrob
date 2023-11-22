#include <gtest/gtest.h>

#include "KnowledgeBaseTest.h"
#include "knowrob/queries/QueryParser.h"

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

TEST_F(KnowledgeBaseTest, atomic_EDB)
{
    auto answerStream = kb_->submitQuery(
            QueryParser::parse("swrl_test:hasSibling(swrl_test:'Ernest', Sibling)"),
            QUERY_FLAG_ALL_SOLUTIONS);
    //auto q = answerStream->createQueue()->toArray();
    EXPECT_EQ(1, 0);
}
