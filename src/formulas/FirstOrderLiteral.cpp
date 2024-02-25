/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/FirstOrderLiteral.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

FirstOrderLiteral::FirstOrderLiteral(const PredicatePtr &predicate, bool isNegative)
		: predicate_(predicate),
		  isNegated_(isNegative) {}

/*
FirstOrderLiteral::FirstOrderLiteral(const FirstOrderLiteral &other, const Substitution &sub)
		: predicate_(std::static_pointer_cast<Predicate>(applyBindings(other.predicate_, sub))),
		  isNegated_(other.isNegated_) {}
*/

std::ostream &FirstOrderLiteral::write(std::ostream &os) const {
	if (isNegated()) {
		os << "not(" << *predicate_ << ")";
	} else {
		os << *predicate_;
	}
	return os;
}

namespace knowrob::py {
	template<>
	void createType<FirstOrderLiteral>() {
		using namespace boost::python;
		class_<FirstOrderLiteral, std::shared_ptr<FirstOrderLiteral>>
				("FirstOrderLiteral", init<const PredicatePtr &, bool>())
				.def("predicate", &FirstOrderLiteral::predicate, return_value_policy<copy_const_reference>())
				.def("isNegated", &FirstOrderLiteral::isNegated)
				.def("functor", &FirstOrderLiteral::functor, return_value_policy<copy_const_reference>())
				.def("arity", &FirstOrderLiteral::arity)
				.def("numVariables", &FirstOrderLiteral::numVariables);
	}
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::FirstOrderLiteral &l) { return l.write(os); }
}
