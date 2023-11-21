#include <gtest/gtest.h>

#include "KnowledgeBaseTest.h"

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

void KnowledgeBaseTest::TearDown() {
	resetDB();
}

void KnowledgeBaseTest::resetDB() {
	// TODO delete all records that are stored in the DB.
}

TEST_F(KnowledgeBaseTest, ThisFails)
{
    EXPECT_EQ(1, 0);
}
