/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PY_CONVERTER_VECTOR_H
#define KNOWROB_PY_CONVERTER_VECTOR_H

namespace knowrob::py {
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
}

#endif //KNOWROB_PY_CONVERTER_VECTOR_H
