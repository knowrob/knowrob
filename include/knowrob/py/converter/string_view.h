/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PY_CONVERTER_STRING_VIEW_H
#define KNOWROB_PY_CONVERTER_STRING_VIEW_H

namespace knowrob::py {
	// This is needed for python to work with std::string_view.
	struct string_view_to_python_str {
		static PyObject *convert(std::string_view s) {
			return PyUnicode_FromStringAndSize(s.data(), s.size());
		}
	};

	// This is needed to map Python strings into std::string_view.
	struct python_str_to_string_view {
		static void *convertible(PyObject *obj_ptr) {
			if (!PyUnicode_Check(obj_ptr)) return 0;
			return obj_ptr;
		}

		static void construct(PyObject *obj_ptr, boost::python::converter::rvalue_from_python_stage1_data *data) {
			const char *value = PyUnicode_AsUTF8(obj_ptr);
			if (value == nullptr) boost::python::throw_error_already_set();
			void *storage = ((boost::python::converter::rvalue_from_python_storage<std::string_view> *) data)->storage.bytes;
			new(storage) std::string_view(value);
			data->convertible = storage;
		}
	};

	void register_string_view_converter() {
		boost::python::to_python_converter<std::string_view, string_view_to_python_str>();
		boost::python::converter::registry::push_back(
				&python_str_to_string_view::convertible,
				&python_str_to_string_view::construct,
				boost::python::type_id<std::string_view>()
		);
	}
}

#endif //KNOWROB_PY_CONVERTER_STRING_VIEW_H
