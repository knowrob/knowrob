/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PY_UTILS_H
#define KNOWROB_PY_UTILS_H

#include "string"
#include <boost/python.hpp>
#include "knowrob/py/utils.h"
#include "knowrob/reasoner/ReasonerError.h"
#include "knowrob/Logger.h"

namespace knowrob::py {
	void handlePythonError() {
		PyObject *p_type, *p_value, *p_traceback;

		PyErr_Fetch(&p_type, &p_value, &p_traceback);
		PyErr_NormalizeException(&p_type, &p_value, &p_traceback);
		Py_XDECREF(p_type);
		Py_XDECREF(p_traceback);

		PyObject* pyExcValueStr1 = PyObject_Repr(p_value);
		PyObject* pyExcValueStr2 = PyUnicode_AsEncodedString(pyExcValueStr1, "utf-8", "Error ~");
		std::string strExcValue(PyBytes_AS_STRING(pyExcValueStr2));
		Py_XDECREF(p_value);
		Py_XDECREF(pyExcValueStr1);
		Py_XDECREF(pyExcValueStr2);

		throw ReasonerError(strExcValue);
	}
}

#endif //KNOWROB_PY_UTILS_H
