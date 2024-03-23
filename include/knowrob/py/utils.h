/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PY_UTILS_H
#define KNOWROB_PY_UTILS_H

#include <boost/python.hpp>
#include "PythonError.h"

namespace knowrob::py {
	// call a method of a python object
	template<typename R, typename... Args> R call_method(PyObject *self, const char *method, Args... args) {
		try {
			return boost::python::call_method<R>(self, method, boost::python::object(args)...);
		} catch(const boost::python::error_already_set&) {
			throw PythonError();
		}
	}

	/**
	 * Call a function and translate boost::python::error_already_set exceptions into PythonError.
	 * @param goal the function to call.
	 */
	template<typename R> R call(const std::function<R()>& goal) {
		try {
			return goal();
		} catch(const boost::python::error_already_set&) {
			throw PythonError();
		}
	}

	/**
	 * Resolve a module path to a file path.
	 * @param modulePath the module path.
	 * @return the file path.
	 */
	std::string resolveModulePath(std::string_view modulePath);

	/**
	 * A template function to create a new type in Python.
	 * @tparam T The C++ type to map into Python.
	 */
	template <typename T> void createType();
}

#endif //KNOWROB_PY_UTILS_H
