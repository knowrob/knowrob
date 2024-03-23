/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/PredicateIndicator.h"
#include "knowrob/terms/Function.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

bool PredicateIndicator::operator==(const PredicateIndicator &other) const {
	return arity_ == other.arity_ && *functor_ == *other.functor_;
}

bool PredicateIndicator::operator<(const PredicateIndicator &other) const {
	return (other.functor_->stringForm() < this->functor_->stringForm()) ||
		   (other.arity_ < this->arity_);
}

void PredicateIndicator::write(std::ostream &os) const {
	os << *functor_ << '/' << arity_;
}

std::shared_ptr<Term> PredicateIndicator::toTerm() const {
	return std::make_shared<Function>(Function("/", {
			functor(),
			std::make_shared<Long>(arity())
	}));
}

namespace knowrob::py {
	template<>
	void createType<PredicateIndicator>() {
		using namespace boost::python;
		class_<PredicateIndicator, std::shared_ptr<PredicateIndicator>>
				("PredicateIndicator", init<const std::string &, unsigned int>())
				.def("__eq__", &PredicateIndicator::operator==)
				.def(self < self)
				.def("name", &PredicateIndicator::functor, return_value_policy<copy_const_reference>())
				.def("arity", &PredicateIndicator::arity)
				.def("toTerm", &PredicateIndicator::toTerm);
	}
}
