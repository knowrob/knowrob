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
		Py_XDECREF(p_type);
		Py_XDECREF(p_traceback);

		// Translate p_traceback to std::string
		PyObject* modules = PyImport_ImportModule("traceback");
		PyObject* pyExcTracebackStr = PyObject_CallMethod(modules, "format_exception", "OOO", p_type, p_value, p_traceback);

		PyObject* pyExcTracebackStrJoined = PyUnicode_Join(PyUnicode_FromString(""), pyExcTracebackStr);
		PyObject* pyExcTracebackStrBytes = PyUnicode_AsEncodedString(pyExcTracebackStrJoined, "utf-8", "ignore");
		std::string strExcTraceback(PyBytes_AS_STRING(pyExcTracebackStrBytes));

		PyObject* pyExcValueStr1 = PyObject_Repr(p_value);
		PyObject* pyExcValueStr2 = PyUnicode_AsEncodedString(pyExcValueStr1, "utf-8", "Error ~");
		std::string strExcValue(PyBytes_AS_STRING(pyExcValueStr2));
		Py_XDECREF(p_value);
		Py_XDECREF(pyExcValueStr1);
		Py_XDECREF(pyExcValueStr2);

		throw ReasonerError(strExcValue, strExcTraceback);
	}
}

#endif //KNOWROB_PY_UTILS_H
