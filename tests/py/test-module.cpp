/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include <boost/python/import.hpp>
#include <knowrob/terms/Atom.h>
#include <boost/python/extract.hpp>
#include "knowrob/Logger.h"
#include "knowrob/py/utils.h"
#include "knowrob/terms/String.h"
#include "knowrob/triples/FramedTriple.h"

namespace python = boost::python;
using namespace knowrob;

class BoostPythonTests : public testing::Test {
protected:
	static python::object test_module;
	static python::object knowrob_module;
	static python::object AssertionError;

	// Per-test-suite set-up.
	static void SetUpTestSuite() {
		try {
			py::call<void>([&] {
				// make sure the knowrob module is loaded, without it conversion of types won't work.
				knowrob_module = python::import("knowrob");
				test_module = python::import("tests.py.test_boost_python");
			});
		} catch (const std::exception& e) {
			KB_ERROR("Failed to set up test suite. {}", e.what());
		}
	}

	static python::object do_call(std::string_view file, uint32_t line, std::string_view method_name, const std::function<python::object(python::object&)> &gn) {
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

python::object BoostPythonTests::test_module;
python::object BoostPythonTests::knowrob_module;

#define EXPECT_CONVERTIBLE_TO_PY(x) EXPECT_NO_THROW( EXPECT_FALSE( \
	py::call<bool>([&]{ return boost::python::object(x).is_none(); })))
#define BOOST_TEST_CALL0(method_name, ...) call(__FILE__, __LINE__, method_name, __VA_ARGS__)
#define BOOST_TEST_CALL1(method_name) call(__FILE__, __LINE__, method_name)

TEST_F(BoostPythonTests, atom_to_python) {
	// test that we can create a term in C++ and pass it to Python.
	auto atom = Atom::Tabled("hello");
	// terms can be passed by reference or by value
	EXPECT_CONVERTIBLE_TO_PY(atom);
	EXPECT_CONVERTIBLE_TO_PY(*atom);
	// pass atom into Python code and inspect it there
	EXPECT_NO_THROW(BOOST_TEST_CALL0("atom_to_python", python::object(atom)));
	EXPECT_NO_THROW(BOOST_TEST_CALL0("atom_to_python", python::object(*atom)));
}

TEST_F(BoostPythonTests, string_copy_from_python) {
	python::object s = BOOST_TEST_CALL1("string_copy_from_python");
	EXPECT_FALSE(boost::python::object(s).is_none());
	auto extracted = boost::python::extract<String>(s);
	EXPECT_TRUE(extracted.check());
	if(extracted.check()) {
		const auto& str = extracted();
		EXPECT_EQ(str.stringForm(), "hello");
	}
}

TEST_F(BoostPythonTests, modify_triple_in_python) {
	// test that we can create a triple in C++ and pass it to Python where it is modified.
	// Note: it might not be safe to pass a FramedTripleView into Python to
	//       fill it with string data as the Python strings might be garbage collected
	//       at some point after returning the call.
	auto triple = std::make_shared<FramedTripleCopy>(
			"hello", "knows", "world");
	// pass triple into Python code and modify it there
	EXPECT_NO_THROW(BOOST_TEST_CALL0("modify_triple_in_python", python::object(triple)));
	EXPECT_EQ(triple->subject(), "olleh");
	EXPECT_EQ(triple->predicate(), "swonk");
}

TEST_F(BoostPythonTests, optionals) {
	std::optional<std::string_view> opt_str_view;
	std::optional<XSDType> opt_xsd_type;
	std::optional<double> opt_double;
	EXPECT_NO_THROW(BOOST_TEST_CALL0("optional_is_none", python::object(opt_str_view)));
	EXPECT_NO_THROW(BOOST_TEST_CALL0("optional_is_none", python::object(opt_xsd_type)));
	EXPECT_NO_THROW(BOOST_TEST_CALL0("optional_is_none", python::object(opt_double)));
	opt_xsd_type = XSDType::STRING;
	EXPECT_NO_THROW(BOOST_TEST_CALL0("optional_is_not_none", python::object(opt_xsd_type)));
	EXPECT_NO_THROW(BOOST_TEST_CALL0("set_xsd_optional", python::object(opt_xsd_type)));
	// value is copied, so the original value should not change
	EXPECT_EQ(opt_xsd_type, XSDType::STRING);
}
