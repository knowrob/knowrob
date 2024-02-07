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
	class PrologTestsBase: public testing::Test {
	protected:
		static void runPrologTests(const std::shared_ptr<knowrob::PrologReasoner> &reasoner,
								   const std::string &target);
	};

	template <class T> class PrologTests: public PrologTestsBase {
	protected:
		// Per-test-suite set-up.
		static void SetUpTestSuite() { reasoner();  }

		static void runTests(const std::string &t) { runPrologTests(reasoner(), t); }

		static std::shared_ptr<T> reasoner() {
			static std::shared_ptr<T> r;
			static std::shared_ptr<BackendManager> backendManager;
			static std::shared_ptr<ReasonerManager> reasonerManager;
			static int reasonerIndex_=0;
			if(!r) {
			    backendManager = std::make_shared<BackendManager>(nullptr);
			    reasonerManager = std::make_shared<ReasonerManager>(nullptr, backendManager);

				std::stringstream ss;
				ss << "prolog" << reasonerIndex_++;
				r = std::make_shared<T>();
                reasonerManager->addReasoner(ss.str(), r);
				r->loadConfig(knowrob::ReasonerConfig());
			}
			return r;
		}
	};
}

#endif //KNOWROB_PROLOG_TESTS_H_
