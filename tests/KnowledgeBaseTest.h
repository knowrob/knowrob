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
    // some constants used in the tests
	static std::shared_ptr<knowrob::StringTerm> Fred_;
	static std::shared_ptr<knowrob::StringTerm> Ernest_;
	static std::shared_ptr<knowrob::StringTerm> Lea_;
	static std::shared_ptr<knowrob::StringTerm> Rex_;
	static std::shared_ptr<knowrob::StringTerm> hasSibling_;
	static std::shared_ptr<knowrob::StringTerm> hasNumber_;
	static std::shared_ptr<knowrob::StringTerm> hasAncestor_;
	static std::shared_ptr<knowrob::StringTerm> p_;
	static std::shared_ptr<knowrob::StringTerm> q_;
	static std::shared_ptr<knowrob::Variable> varX_;
	static std::shared_ptr<knowrob::Variable> varY_;
	static std::shared_ptr<knowrob::Variable> varZ_;
	static std::shared_ptr<knowrob::Variable> varNum_;
protected:
    static void SetUpTestSuite();
    //void TearDown() override;
};

#endif //KNOWROB_KNOWLEDGE_BASE_TEST_H
