/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PY_CONVERTER_SHARED_PTR_H
#define KNOWROB_PY_CONVERTER_SHARED_PTR_H

namespace boost {
	// This is needed for boost::python to work with std::shared_ptr.
	// @see https://stackoverflow.com/questions/46435509/boostpython-stdshared-ptr-to-stdshared-ptr
	template<class T>
	T *get_pointer(std::shared_ptr<T> p) { return p.get(); }
}

#endif //KNOWROB_PY_CONVERTER_SHARED_PTR_H
