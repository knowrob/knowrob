/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "sstream"
#include "knowrob/terms/Term.h"
#include "knowrob/terms/Atomic.h"
#include "knowrob/terms/Function.h"
#include "knowrob/terms/Variable.h"
#include "knowrob/integration/python/utils.h"
#include "knowrob/knowrob.h"

using namespace knowrob;

const std::set<std::string_view> Term::noVariables_ = {};

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

size_t Term::hash() const {
	size_t val = 0;
	hashCombine(val, uint8_t(termType()));
	switch (termType()) {
		case TermType::ATOMIC:
			hashCombine(val, ((Atomic *) this)->hashOfAtomic());
			break;
		case TermType::FUNCTION:
			hashCombine(val, ((Function *) this)->hashOfFunction());
			break;
		case TermType::VARIABLE:
			hashCombine(val, std::hash<std::string_view>{}(((Variable *) this)->name()));
			break;
	}
	return val;
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
		explicit TermWrap(PyObject *p, TermType termType) : self(p), Term(termType) {}

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
				.value("VARIABLE", TermType::VARIABLE)
				.export_values();
		class_<Term, std::shared_ptr<TermWrap>, boost::noncopyable>
				("Term", no_init)
				.def("__eq__", &Term::operator==)
				.def("__repr__", +[](Term &t) { return readString(t); })
				.def("__hash__", &Term::hash)
				.def("termType", &Term::termType)
				.def("isAtomic", &Term::isAtomic)
				.def("isAtom", &Term::isAtom)
				.def("isVariable", &Term::isVariable)
				.def("isFunction", &Term::isFunction)
				.def("isNumeric", &Term::isNumeric)
				.def("isString", &Term::isString)
				.def("isIRI", &Term::isIRI)
				.def("isBlank", &Term::isBlank)
				.def("isGround", &Term::isGround)
				.def("variables", pure_virtual(&Term::variables), return_value_policy<copy_const_reference>());
		// register shared_ptr to Term
		register_ptr_to_python< std::shared_ptr< Term > >();
	}
}
