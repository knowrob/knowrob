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

namespace knowrob::py {
	void handlePythonError() {
		PyObject *p_type, *p_value, *p_traceback;
		PyErr_Fetch(&p_type, &p_value, &p_traceback);
		// TODO: could include line number
		// TODO: could include stacktrace in exception
		std::string strErrorMessage = boost::python::extract<std::string>(p_value);
		throw ReasonerError(strErrorMessage);
	}
}

#endif //KNOWROB_PY_UTILS_H
