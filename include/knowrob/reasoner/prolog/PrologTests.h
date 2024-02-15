/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PROLOG_TESTS_H_
#define KNOWROB_PROLOG_TESTS_H_

#include <gtest/gtest.h>

#include "PrologReasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/Logger.h"

namespace knowrob {
	/**
	 * A baseclass for prolog test fixtures.
	 */
	class PrologTestsBase : public testing::Test {
	protected:
		static void runPrologTests(
				const std::shared_ptr<knowrob::PrologReasoner> &reasoner,
				const std::string &target);
	};

	template<class ReasonerType, class BackendType>
	class PrologTests : public PrologTestsBase {
	protected:
		// Per-test-suite set-up.
		static void SetUpTestSuite() {
			// Initialize the reasoner
			try {
				reasoner();
			} catch (std::exception &e) {
				FAIL() << "SetUpTestSuite failed: " << e.what();
			}
		}

		static void runTests(const std::string &t) {
			try {
				runPrologTests(reasoner(), t);
			} catch (std::exception &e) {
				FAIL() << "runTests failed: " << e.what();
			}
		}

		static std::shared_ptr<ReasonerType> reasoner() {
			static std::shared_ptr<ReasonerType> r;
			static std::shared_ptr<BackendType> db;
			static std::shared_ptr<KnowledgeBase> knowledgeBase;
			static int reasonerIndex_ = 0;
			if (!r) {
				knowledgeBase = std::make_shared<KnowledgeBase>();

				std::stringstream ss;
				ss << "prolog" << reasonerIndex_++;

				db = std::make_shared<BackendType>();
				knowledgeBase->backendManager()->addBackend(ss.str(), db);
				db->loadConfig(knowrob::ReasonerConfig());

				r = std::make_shared<ReasonerType>();
				r->setDataBackend(db);
				knowledgeBase->reasonerManager()->addReasoner(ss.str(), r);
				r->loadConfig(knowrob::ReasonerConfig());
			}
			return r;
		}

		static std::shared_ptr<KnowledgeBase> kb() {
			return reasoner()->reasonerManager().kb();
		}
	};
}

#endif //KNOWROB_PROLOG_TESTS_H_
