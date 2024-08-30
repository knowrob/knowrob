/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */
#ifndef MODULENAME
#define MODULENAME knowrob
#endif
// Macro to convert another macro's expansion to a string
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#include <gtest/gtest.h>
#include <boost/python/import.hpp>
#include <knowrob/terms/Atom.h>
#include <boost/python/extract.hpp>
#include "knowrob/Logger.h"
#include "knowrob/integration/python/utils.h"
#include "knowrob/terms/String.h"
#include "knowrob/triples/FramedTriple.h"
#include "knowrob/URI.h"

namespace python = boost::python;
using namespace knowrob;

class JupyterTests : public testing::Test {
protected:
	static python::object test_module;
	static python::object knowrob_module;
	static python::object AssertionError;


	// Function to return the module name as a string
	static constexpr const char *moduleNameStr() {
		return TOSTRING(MODULENAME);
	}

	// Per-test-suite set-up.
	static void SetUpTestSuite() {
		try {
			py::call_with_gil<void>([&] {
				// make sure the knowrob module is loaded, without it conversion of types won't work.
				// Conditionally import module based on MODULENAME
				if (std::string(moduleNameStr()) == "knowrob") {
					knowrob_module = python::import("knowrob");
				} else {
					std::string fullModuleName = std::string("knowrob.") + moduleNameStr();
					knowrob_module = python::import(fullModuleName.c_str());
				}
				test_module = python::import("tests.py.test_jupyter");
			});
		} catch (const std::exception& e) {
			KB_ERROR("Failed to set up test suite. {}", e.what());
		}
	}

	static python::object do_call(std::string_view file, uint32_t line, std::string_view method_name, const std::function<python::object(python::object &)> &gn) {
		EXPECT_FALSE(test_module.is_none());
		if (test_module.is_none()) { return {}; }

		python::object fn = test_module.attr(method_name.data());
		EXPECT_FALSE(fn.is_none());
		if (fn.is_none()) { return {}; }

		try {
			return py::call<python::object>([&] { return gn(fn); });
		} catch (const PythonError &err) {
			GTEST_MESSAGE_AT_(file.data(), line, method_name.data(), testing::TestPartResult::kNonFatalFailure) << err.what();
		}
		return {};
	}

	static python::object call(std::string_view file, uint32_t line, std::string_view method_name, const python::object& args...) {
		return do_call(
			file, line, method_name,
			[&](python::object &fn) { return fn(args); });
	}

	static python::object call(std::string_view file, uint32_t line, std::string_view method_name) {
		return do_call(
			file, line, method_name,
			[&](python::object &fn) { return fn(); });
	}
};

python::object JupyterTests::test_module;
python::object JupyterTests::knowrob_module;

#define BOOST_TEST_CALL0(method_name, ...) call(__FILE__, __LINE__, method_name, __VA_ARGS__)
#define BOOST_TEST_CALL1(method_name) call(__FILE__, __LINE__, method_name)

#define TEST_JUPYTER(notebook) {  \
    py::gil_lock lock;            \
    EXPECT_NO_THROW(BOOST_TEST_CALL0("test_notebook", python::object(URI::resolve(notebook)))); }

TEST_F(JupyterTests, python_kb) { TEST_JUPYTER("jupyter/python-kb.ipynb"); }
