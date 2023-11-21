
#include <gtest/gtest.h>

// fixture class for testing
class KnowledgeBaseTest : public ::testing::Test {
protected:
    //static std::shared_ptr<KnoweldgeBase> kb_;
    static void SetUpTestSuite() {
        // setup, once called
    }
    // void TearDown() override {}
    // add some helper functions below...
};

// TODO:
// here main interface of KnoweldgeBase should be tested, in particular:
// - submitQuery*
// - insert*
//
// what is needed:
// - setup database backend for testing, insert tmp data on which queries can be evaluated
// - generate an adhoc configuration for KnowledgeBase. currently this must be a boost property tree
// - use some dummy reasoner with fixed input-output mapping that can be tested easily
//

TEST_F(KnowledgeBaseTest, ThisFails)
{
    EXPECT_EQ(1, 0);
}
