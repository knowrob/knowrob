/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "PythonTests.h"
#include <knowrob/terms/Atom.h>
#include <boost/python/extract.hpp>
#include "knowrob/terms/String.h"
#include "knowrob/semweb/Triple.h"

namespace python = boost::python;
using namespace knowrob;

class PythonApplicationTests : public PythonTests {
protected:
	// Per-test-suite set-up.
	static void SetUpTestSuite() {
		PythonTests::SetUpTestSuite("tests.py.test_application");
	}
};

#define EXPECT_CONVERTIBLE_TO_PY(x) EXPECT_NO_THROW( EXPECT_FALSE( \
    py::call_with_gil<bool>([&]{ return boost::python::object(x).is_none(); })))

TEST_F(PythonApplicationTests, kb_assert) {
	std::string testfile = "tests/settings/kb-test.json";
	EXPECT_NO_THROW(PYTHON_TEST_CALL1("ka_assert"));
}
