/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PY_UTILS_H
#define KNOWROB_PY_UTILS_H

#include <boost/python.hpp>

namespace knowrob::py {
	void handlePythonError();

	// call a method of a python object
	template<typename R, typename... Args> R call_method(PyObject *self, const char *method, Args... args) {
		try {
			return boost::python::call_method<R>(self, method, boost::python::object(args)...);
		} catch (const boost::python::error_already_set&) {
			knowrob::py::handlePythonError();
		}
	}

	template <typename T> void createType();
}

#endif //KNOWROB_PY_UTILS_H
