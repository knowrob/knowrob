
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

TEST_F(KnowledgeBaseTest, ThisFails)
{
    EXPECT_EQ(1, 0);
}
