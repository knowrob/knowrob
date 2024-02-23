/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "sstream"
#include "knowrob/terms/Term.h"
#include "knowrob/terms/Atomic.h"
#include "knowrob/terms/Function.h"
#include "knowrob/terms/Variable.h"

using namespace knowrob;

const VariableSet Term::noVariables_ = {};

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

bool VariableComparator::operator()(const Variable *const &v0, const Variable *const &v1) const {
	return *v0 < *v1;
}

namespace std {
	ostream &operator<<(ostream &os, const knowrob::Term &t) { //NOLINT
		knowrob::TermWriter(t,os);
		return os;
	}
}
