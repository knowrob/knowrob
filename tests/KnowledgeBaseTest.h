//
// Created by daniel on 21.11.23.
//

#ifndef KNOWROB_KNOWLEDGE_BASE_TEST_H
#define KNOWROB_KNOWLEDGE_BASE_TEST_H

#include "knowrob/KnowledgeBase.h"
#include "knowrob/terms/IRIAtom.h"

// fixture class for testing
class KnowledgeBaseTest : public ::testing::Test {
public:
    static std::shared_ptr<knowrob::KnowledgeBase> kb_;
    // some constants used in the tests
	static std::shared_ptr<knowrob::IRIAtom> Fred_;
	static std::shared_ptr<knowrob::IRIAtom> Ernest_;
	static std::shared_ptr<knowrob::IRIAtom> Lea_;
	static std::shared_ptr<knowrob::IRIAtom> Rex_;
	static std::shared_ptr<knowrob::IRIAtom> hasSibling_;
	static std::shared_ptr<knowrob::IRIAtom> hasNumber_;
	static std::shared_ptr<knowrob::IRIAtom> hasAncestor_;
	static std::shared_ptr<knowrob::IRIAtom> p_;
	static std::shared_ptr<knowrob::IRIAtom> q_;
	static std::shared_ptr<knowrob::Variable> varX_;
	static std::shared_ptr<knowrob::Variable> varY_;
	static std::shared_ptr<knowrob::Variable> varZ_;
	static std::shared_ptr<knowrob::Variable> varNum_;
protected:
    static void SetUpTestSuite();
    //void TearDown() override;
};

#endif //KNOWROB_KNOWLEDGE_BASE_TEST_H
