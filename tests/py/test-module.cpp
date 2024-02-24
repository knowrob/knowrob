
#include <gtest/gtest.h>
#include <boost/python/import.hpp>
#include <knowrob/terms/Atom.h>
#include <boost/python/extract.hpp>
#include <boost/python/tuple.hpp>
#include "knowrob/Logger.h"
#include "knowrob/py/utils.h"
#include "knowrob/terms/String.h"
#include "knowrob/semweb/FramedTriple.h"

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
			// make sure the knowrob module is loaded, without it conversion of types won't work.
			knowrob_module = python::import("knowrob");
			test_module = python::import("tests.py.test_boost_python");
		} catch (const python::error_already_set&) {
			knowrob::py::handlePythonError();
		}
	}

	static python::object call(std::string_view method_name, const python::object& args...) {
		EXPECT_FALSE(test_module.is_none());
		if (test_module.is_none()) { return python::tuple(); }

		python::object fn = test_module.attr(method_name.data());
		EXPECT_FALSE(fn.is_none());
		if (fn.is_none()) { return python::tuple(); }

		try {
			return fn(args);
		} catch (const boost::python::error_already_set&) {
			handle_exception(method_name);
		}
		return python::tuple();
	}

	static python::object call(std::string_view method_name) {
		EXPECT_FALSE(test_module.is_none());
		if (test_module.is_none()) { return python::tuple(); }

		python::object fn = test_module.attr(method_name.data());
		EXPECT_FALSE(fn.is_none());
		if (fn.is_none()) { return python::tuple(); }

		try {
			return fn();
		} catch (const boost::python::error_already_set&) {
			handle_exception(method_name);
		}
		return python::tuple();
	}

	static void handle_exception(std::string_view method_name) {
			PyObject *ptype, *pvalue, *ptraceback;
			PyErr_Fetch(&ptype, &pvalue, &ptraceback);
			PyErr_NormalizeException(&ptype, &pvalue, &ptraceback);
			python::handle<> hType(ptype);
			python::object extype(hType);
			python::handle<> hTraceback(ptraceback);
			python::object traceback(hTraceback);

			long line_number = -1;
			if (traceback) {
				auto e_line_number = python::extract<long>(traceback.attr("tb_lineno"));
				if (e_line_number.check()) {
					line_number = e_line_number;
				}
			}
			if (line_number == -1) {
				KB_WARN("Failed to extract line number from Python exception");
				line_number = 0;
			}

			std::string file_path;
			if (traceback) {
				auto e_file_path = python::extract<std::string>(traceback.
						attr("tb_frame").
						attr("f_code").
						attr("co_filename"));
				if(e_file_path.check()) file_path = e_file_path;
			}
			if (file_path.empty()) {
				KB_WARN("Failed to extract file path from Python exception");
				file_path = ".";
			}

			std::string error_message;
			if (pvalue) {
				auto e_error_message = python::extract<std::string>(pvalue);
				if(e_error_message.check()) {
					// try to extract directly encoded message first
					error_message = e_error_message;
				} else {
					// try to extract message from handle, e.g. AssertionError
					boost::python::handle<> phandle(pvalue);
					python::object py_obj(phandle);
					python::object py_obj_args = py_obj.attr("args");
					if (len(py_obj_args) > 0) {
						auto e_message = python::extract<std::string>(
									py_obj_args.attr("__getitem__")(0));
						if(e_message.check()) error_message = e_message;
					}
				}
			}
			if (error_message.empty()) {
				error_message = "Unknown error";
			}

			GTEST_MESSAGE_AT_(file_path.c_str(),
			                  line_number,
			                  method_name.data(),
			                  testing::TestPartResult::kNonFatalFailure) <<
			                  error_message.c_str();
	}
};

python::object BoostPythonTests::test_module;
python::object BoostPythonTests::knowrob_module;

#define EXPECT_CONVERTIBLE_TO_PY(x) \
	EXPECT_NO_THROW(EXPECT_FALSE(boost::python::object(x).is_none()))

TEST_F(BoostPythonTests, atom_to_python) {
	// test that we can create a term in C++ and pass it to Python.
	auto atom = Atom::Tabled("hello");
	// terms can be passed by reference or by value
	EXPECT_CONVERTIBLE_TO_PY(atom);
	EXPECT_CONVERTIBLE_TO_PY(*atom);
	// pass atom into Python code and inspect it there
	EXPECT_NO_THROW(call("atom_to_python", python::object(atom)));
	EXPECT_NO_THROW(call("atom_to_python", python::object(*atom)));
}

TEST_F(BoostPythonTests, string_copy_from_python) {
	python::object s = call("string_copy_from_python");
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
	EXPECT_NO_THROW(call("modify_triple_in_python", python::object(triple)));
	EXPECT_EQ(triple->subject(), "olleh");
	EXPECT_EQ(triple->predicate(), "swonk");
}

TEST_F(BoostPythonTests, optionals) {
	std::optional<std::string_view> opt_str_view;
	std::optional<XSDType> opt_xsd_type;
	std::optional<double> opt_double;
	EXPECT_NO_THROW(call("optional_is_none", python::object(opt_str_view)));
	EXPECT_NO_THROW(call("optional_is_none", python::object(opt_xsd_type)));
	EXPECT_NO_THROW(call("optional_is_none", python::object(opt_double)));
	opt_xsd_type = XSDType::STRING;
	EXPECT_NO_THROW(call("optional_is_not_none", python::object(opt_xsd_type)));
	EXPECT_NO_THROW(call("set_xsd_optional", python::object(opt_xsd_type)));
	// value is copied, so the original value should not change
	EXPECT_EQ(opt_xsd_type, XSDType::STRING);
}
