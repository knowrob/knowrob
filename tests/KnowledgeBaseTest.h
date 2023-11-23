//
// Created by daniel on 21.11.23.
//

#ifndef KNOWROB_KNOWLEDGE_BASE_TEST_H
#define KNOWROB_KNOWLEDGE_BASE_TEST_H

#include "knowrob/KnowledgeBase.h"

// fixture class for testing
class KnowledgeBaseTest : public ::testing::Test {
public:
    static std::shared_ptr<knowrob::KnowledgeBase> kb_;
protected:
    static void SetUpTestSuite();
    //void TearDown() override;
};

#endif //KNOWROB_KNOWLEDGE_BASE_TEST_H
