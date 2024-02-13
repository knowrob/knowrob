
#ifndef KNOWROB_PY_CONVERTER_H
#define KNOWROB_PY_CONVERTER_H

#include <boost/python.hpp>
#include "optional"
#include "memory"

// This header contains converter classes for the mapping of C++ classes to Python.
// Widely based on coe pieces found on the Internet.
// In particular, the file contains converters for std::shared_ptr, std::optional, and std::vector.

using namespace boost::python;

namespace boost {
	// This is needed for boost::python to work with std::shared_ptr.
	// @see https://stackoverflow.com/questions/46435509/boostpython-stdshared-ptr-to-stdshared-ptr
	template<class T>
	T *get_pointer(std::shared_ptr<T> p) { return p.get(); }
}

/*** python-list to c++-list */
template<typename containedType>
struct custom_vector_from_seq {
	custom_vector_from_seq() {
		boost::python::converter::registry::push_back(&convertible,
													  &construct,
													  boost::python::type_id<std::vector<containedType> >());
	}

	static void *convertible(PyObject *obj_ptr) {
		// the second condition is important, for some reason otherwise there were attempted conversions
		// of Body to list which failed afterwards.
		if (!PySequence_Check(obj_ptr) || !PyObject_HasAttrString(obj_ptr, "__len__")) return nullptr;
		return obj_ptr;
	}

	static void construct(PyObject *obj_ptr, boost::python::converter::rvalue_from_python_stage1_data *data) {
		void *storage = ((boost::python::converter::rvalue_from_python_storage<std::vector<containedType> > *) (data))->storage.bytes;
		new(storage) std::vector<containedType>();
		auto v = (std::vector<containedType> *) (storage);
		auto l = PySequence_Size(obj_ptr);
		if (l < 0) abort();
		v->reserve(l);
		for (int i = 0; i < l; i++) {
			v->push_back(boost::python::extract<containedType>(PySequence_GetItem(obj_ptr, i)));
		}
		data->convertible = storage;
	}
};

/** handling of std::optional, map no-value to None in Python. */
template<typename T>
struct python_optional : private boost::noncopyable {
	struct conversion : public boost::python::converter::expected_from_python_type<T> {
		static PyObject *convert(std::optional<T> const &value) {
			using namespace boost::python;
			return incref((value ? object(*value) : object()).ptr());
		}
	};

	static void *convertible(PyObject *obj) {
		using namespace boost::python;
		return obj == Py_None || extract<T>(obj).check() ? obj : NULL;
	}

	static void constructor(
			PyObject *obj,
			boost::python::converter::rvalue_from_python_stage1_data *data
	) {
		using namespace boost::python;
		void *const storage =
				reinterpret_cast<
						converter::rvalue_from_python_storage<std::optional<T> > *
						>(data)->storage.bytes;
		if (obj == Py_None) {
			new(storage) std::optional<T>();
		} else {
			new(storage) std::optional<T>(extract<T>(obj));
		}
		data->convertible = storage;
	}

	explicit python_optional() {
		using namespace boost::python;
		if (!extract<std::optional<T> >(object()).check()) {
			to_python_converter<std::optional<T>, conversion, true>();
			converter::registry::push_back(
					&convertible,
					&constructor,
					type_id<std::optional<T> >(),
					&conversion::get_pytype
			);
		}
	}
};

#endif //KNOWROB_PY_CONVERTER_H
