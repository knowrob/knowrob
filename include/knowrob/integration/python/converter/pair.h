/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PY_CONVERTER_PAIR_H
#define KNOWROB_PY_CONVERTER_PAIR_H

namespace knowrob::py {

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

// Converter from C++ pair to Python tuple
	struct PairToTupleConverter {
		static PyObject *
		convert(const std::pair<std::basic_string_view<char, std::char_traits<char> > const, std::pair<std::shared_ptr<knowrob::Variable>, std::shared_ptr<knowrob::Term> > > &pair) {
			return boost::python::incref(
					boost::python::make_tuple(pair.first, pair.second.first, pair.second.second).ptr());
		}
	};

// Register the converter in the module initialization function
	void register_pair_converter() {
		boost::python::to_python_converter<std::pair<std::basic_string_view<char, std::char_traits<char> > const, std::pair<std::shared_ptr<knowrob::Variable>, std::shared_ptr<knowrob::Term> > >, PairToTupleConverter>();
	}
}

#endif //KNOWROB_PY_CONVERTER_PAIR_H
