/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/algorithm/string/replace.hpp>
#include "knowrob/Logger.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/terms/OptionList.h"
#include "knowrob/reasoner/prolog/PrologTests.h"
#include "knowrob/reasoner/prolog/PrologBackend.h"

using namespace knowrob;

static std::string unescapeString(const std::string &input) {
	std::string result = input;
	boost::replace_all(result, "\\n", "\n");
	boost::replace_all(result, "\\t", "\t");
	return result;
}

void PrologTestsBase::runPrologTests(
		const std::shared_ptr<knowrob::PrologReasoner> &reasoner,
		const std::string &target) {
	bool hasResult = false;
	int numTests = 0;
	int numFailedTests = 0;

	for (const auto &t: reasoner->runTests(PrologEngine::getPrologPath(target))) {
		hasResult = true;
		numTests += 1;

		// each result is a term element/3
		ASSERT_EQ(t->type(), TermType::PREDICATE);
		auto *pElem = (Predicate *) t.get();
		ASSERT_EQ(pElem->indicator()->functor(), "element");
		ASSERT_EQ(pElem->indicator()->arity(), 3);
		ASSERT_EQ(*(pElem->arguments()[0]), StringTerm("testcase"));

		// the second argument has the form:
		// [name=::string, file=::string, line=::int, time=::double]
		OptionList trace(pElem->arguments()[1]);
		ASSERT_TRUE(trace.contains("file"));
		ASSERT_TRUE(trace.contains("line"));
		ASSERT_TRUE(trace.contains("name"));
		const auto &name = trace.getString("name", "");
		const char *file = trace.getString("file", "").c_str();
		const long line = trace.getLong("line", 0);

		// the third argument is a list of failures.
		auto *failureList = (ListTerm *) pElem->arguments()[2].get();
		if (!failureList->isNIL()) {
			numFailedTests += 1;
		}
		for (const auto &failureTerm: (*failureList)) {
			ASSERT_EQ(failureTerm->type(), TermType::PREDICATE);
			// each failure is a term element/3
			auto *errElem = (Predicate *) failureTerm.get();
			ASSERT_EQ(errElem->indicator()->functor(), "element");
			ASSERT_EQ(errElem->indicator()->arity(), 3);
			ASSERT_EQ(*(errElem->arguments()[0]), StringTerm("failure"));

			OptionList errOpts(errElem->arguments()[1]);
			ASSERT_TRUE(errOpts.contains("type"));
			ASSERT_TRUE(errOpts.contains("message"));
			auto message = errOpts.getString("type", "") + ": " + errOpts.getString("message", "");

			// the second argument has the form: [ type=error|failure, message='...' ]
			// the third argument is the exact same message, which is a bit strange.
			//ADD_FAILURE_AT(file,line) << message;
			GTEST_MESSAGE_AT_(file, line, ("test: " + name).c_str(), \
                              testing::TestPartResult::kNonFatalFailure) << unescapeString(message);
		}
	}
	EXPECT_TRUE(hasResult);
	KB_INFO1(target.c_str(), 1, "{}/{} tests succeeded.", (numTests - numFailedTests), numTests);
}

class PrologReasonerTests : public PrologTests<knowrob::PrologReasoner, knowrob::PrologBackend> {
protected:
	static std::string getPath(const std::string &filename) {
		return std::filesystem::path("reasoner") / "prolog" / filename;
	}
};

// register test cases of prolog files in this directory (pl or plt file extension)
TEST_F(PrologReasonerTests, semweb) { runTests(getPath("semweb.pl")); }
