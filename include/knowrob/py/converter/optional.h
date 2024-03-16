/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PY_CONVERTER_OPTIONAL_H
#define KNOWROB_PY_CONVERTER_OPTIONAL_H

#include <boost/python.hpp>
#include "optional"

namespace knowrob::py {
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
}

#endif //KNOWROB_PY_CONVERTER_OPTIONAL_H
