/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/python.hpp>
#include "knowrob/py/PythonError.h"
#include "knowrob/Logger.h"

using namespace knowrob;

PythonError::PythonError(ErrorData *errorData)
		: KnowRobError(errorData->exc_type, errorData->exc_msg, errorData->exc_trace),
		  errorData_(errorData) {
	if (errorData_->exc_file.has_value()) {
		setFile(errorData_->exc_file.value());
	}
	if (errorData_->exc_line.has_value()) {
		setLine(errorData_->exc_line.value());
	}
}

PythonError::PythonError()
		: PythonError(makeErrorData()) {
}

PythonError::ErrorData *PythonError::emptyErrorData(ErrorData *errorData) {
	errorData->exc_type = "UnknownError";
	errorData->exc_msg = "";
	errorData->exc_trace = "";
	errorData->exc_file = "";
	errorData->exc_line = 0;
	return errorData;
}

PythonError::ErrorData *PythonError::makeErrorData() {
	using namespace boost::python;

	auto *errorData = new ErrorData();
	if (!PyErr_Occurred()) {
		// no python error occurred
		KB_WARN("PythonError was created even though no error occurred in Python");
		return emptyErrorData(errorData);
	}

	// try to fetch the Python error
	PyObject * py_type, *py_value, *py_traceback;
	PyErr_Fetch(&py_type, &py_value, &py_traceback);
	if (!py_type) {
		KB_WARN("PythonError was created but PyErr_Fetch returned nullptr for error type.");
		return emptyErrorData(errorData);
	}
	PyErr_NormalizeException(&py_type, &py_value, &py_traceback);
	if (py_traceback != nullptr) {
		PyException_SetTraceback(py_value, py_traceback);
	}

	// Get Boost handles to the Python objects so we get an easier API.
	handle<> h_type(py_type);
	handle<> h_value(allow_null(py_value));
	handle<> h_traceback(allow_null(py_traceback));

	// extract name of error type
	auto e_type = extract<std::string>(object(h_type).attr("__name__"));
	if (e_type.check()) {
		errorData->exc_type = e_type();
	} else {
		errorData->exc_type = "UnknownError";
	}

	// extract error msg by using __str__ method of h_value
	if (h_value) {
		auto e_message = extract<std::string>(object(h_value).attr("__str__")());
		if (e_message.check()) {
			errorData->exc_msg = e_message();
		}
	}

	// finally handle traceback if any
	if (h_traceback) {
		object o_traceback(h_traceback);
		// extract the line number from the traceback
		auto tb_lineno = extract<long>(o_traceback.
				attr("tb_lineno"));
		if (tb_lineno.check()) {
			errorData->exc_line = tb_lineno;
		}
		// extract the file path from the traceback
		auto e_file_path = extract<std::string>(o_traceback.
				attr("tb_frame").
				attr("f_code").
				attr("co_filename"));
		if (e_file_path.check()) {
			errorData->exc_file = e_file_path();
		}

		// Import the `traceback` module and use it to format the exception.
		object format_tb = import("traceback").
				attr("format_tb");
		// Extract the formatted traceback using `format_tb`
		object formatted_list = format_tb(o_traceback);
		object formatted_traceback = str("\n").join(formatted_list).slice(0, -1);
		auto str_traceback = extract<std::string>(formatted_traceback);
		if (str_traceback.check()) {
			errorData->exc_trace = str_traceback();
		}
	}

	return errorData;
}
