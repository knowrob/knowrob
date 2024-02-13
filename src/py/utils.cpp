/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PY_UTILS_H
#define KNOWROB_PY_UTILS_H

#include "string"
#include <boost/python.hpp>
#include <iostream>
#include "knowrob/py/utils.h"
#include "knowrob/reasoner/ReasonerError.h"
#include "boost/stacktrace.hpp"

namespace knowrob::py {
	void handlePythonError() {
		PyObject *p_type, *p_value, *p_traceback;

		PyErr_Fetch(&p_type, &p_value, &p_traceback);
		PyErr_NormalizeException(&p_type, &p_value, &p_traceback);

		// Convert the error value to a string
		PyObject* pyExcValueStr1 = PyObject_Repr(p_value);
		PyObject* pyExcValueStr2 = PyUnicode_AsEncodedString(pyExcValueStr1, "utf-8", "Error ~");
		std::string strExcValue(PyBytes_AS_STRING(pyExcValueStr2));

		// Convert the traceback to a string
		PyObject *pyTracebackModule = PyImport_ImportModule("traceback");
		PyObject *pyTracebackList, *pyTracebackStr1, *pyTracebackStr2;
		std::string strTraceback;
		if (p_traceback && pyTracebackModule) {
			pyTracebackList = PyObject_CallMethod(pyTracebackModule, "format_tb", "O", p_traceback);
			pyTracebackStr1 = PyObject_Str(pyTracebackList);
			pyTracebackStr2 = PyUnicode_AsEncodedString(pyTracebackStr1, "utf-8", "Error ~");
			strTraceback = PyBytes_AS_STRING(pyTracebackStr2);
			Py_XDECREF(pyTracebackList);
			Py_XDECREF(pyTracebackStr1);
			Py_XDECREF(pyTracebackStr2);
		}

		// Cleanup
		Py_XDECREF(p_type);
		Py_XDECREF(p_value);
		Py_XDECREF(p_traceback);
		Py_XDECREF(pyExcValueStr1);
		Py_XDECREF(pyExcValueStr2);
		Py_XDECREF(pyTracebackModule);

		// Throw error with traceback
		throw ReasonerError(strExcValue, strTraceback);
	}
}

#endif //KNOWROB_PY_UTILS_H
