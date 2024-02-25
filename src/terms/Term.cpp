/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "sstream"
#include "knowrob/terms/Term.h"
#include "knowrob/terms/Atomic.h"
#include "knowrob/terms/Function.h"
#include "knowrob/terms/Variable.h"
#include "knowrob/py/utils.h"
#include "knowrob/knowrob.h"

using namespace knowrob;

const std::set<std::string_view> Term::noVariables_ = {};

bool Term::isIRI() const {
	return false;
}

bool Term::isBlank() const {
	return false;
}

bool Term::isAtom() const {
	return termType() == TermType::ATOMIC && ((Atomic *) this)->atomicType() == AtomicType::ATOM;
}

bool Term::isVariable() const {
	return termType() == TermType::VARIABLE;
}

bool Term::isFunction() const {
	return termType() == TermType::FUNCTION;
}

bool Term::isNumeric() const {
	return termType() == TermType::ATOMIC && ((Atomic *) this)->atomicType() == AtomicType::NUMERIC;
}

bool Term::isString() const {
	return termType() == TermType::ATOMIC && ((Atomic *) this)->atomicType() == AtomicType::STRING;
}

bool Term::operator==(const Term &other) const {
	if (this == &other) return true;
	if (termType() != other.termType()) return false;
	switch (termType()) {
		case TermType::ATOMIC:
			return ((Atomic *) this)->isSameAtomic(*((Atomic *) &other));
		case TermType::VARIABLE:
			return ((Variable *) this)->isSameVariable(*((Variable *) &other));
		case TermType::FUNCTION:
			return ((Function *) this)->isSameFunction(*((Function *) &other));
	}
	return false;
}

namespace std {
	ostream &operator<<(ostream &os, const knowrob::Term &t) { //NOLINT
		knowrob::TermWriter(t, os);
		return os;
	}
}

namespace knowrob::py {
	// this struct is needed because Term has pure virtual methods
	struct TermWrap : public Term, boost::python::wrapper<Term> {
		explicit TermWrap(PyObject *p) : self(p), Term() {}

		bool isAtomic() const override { return knowrob::py::call_method<bool>(self, "isAtomic"); }

		TermType termType() const override { return knowrob::py::call_method<TermType>(self, "termType"); }

		size_t hash() const override { return knowrob::py::call_method<size_t>(self, "hash"); }

		const std::set<std::string_view> &
		variables() const override { return knowrob::py::call_method<std::set<std::string_view> &>(self, "variables"); }

	private:
		PyObject *self;
	};

	template<>
	void createType<Term>() {
		using namespace boost::python;
		enum_<TermType>("TermType")
				.value("FUNCTION", TermType::FUNCTION)
				.value("ATOMIC", TermType::ATOMIC)
				.value("VARIABLE", TermType::VARIABLE);
		class_<Term, std::shared_ptr<TermWrap>, boost::noncopyable>
				("Term", no_init)
				.def("__eq__", &Term::operator==)
				.def("__str__", +[](Term &t) { return readString(t); })
				.def("termType", &Term::termType)
				.def("isGround", pure_virtual(&Term::isGround))
				.def("isAtomic", pure_virtual(&Term::isAtomic))
				.def("isAtom", &Term::isAtom)
				.def("isVariable", &Term::isVariable)
				.def("isFunction", &Term::isFunction)
				.def("isNumeric", &Term::isNumeric)
				.def("isString", &Term::isString)
				.def("isIRI", &Term::isIRI)
				.def("isBlank", &Term::isBlank)
				.def("hash", pure_virtual(&Term::hash))
				.def("variables", pure_virtual(&Term::variables), return_value_policy<copy_const_reference>());
	}
}
