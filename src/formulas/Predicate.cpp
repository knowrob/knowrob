/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/Predicate.h"
#include "knowrob/terms/Function.h"

using namespace knowrob;

Predicate::Predicate(std::string_view functor, const std::vector<TermPtr> &arguments)
		: Predicate(Atom::Tabled(functor), arguments) {
}

Predicate::Predicate(const AtomPtr &functor, const std::vector<TermPtr> &arguments)
		: Formula(FormulaType::PREDICATE),
		  functor_(functor),
		  arguments_(arguments),
		  variables_(getVariables1()) {
}

bool Predicate::isEqual(const Formula &other) const {
	const auto &x = static_cast<const Predicate &>(other); // NOLINT
	if (*functor() == *x.functor() && arguments_.size() == x.arguments_.size()) {
		for (int i = 0; i < arguments_.size(); ++i) {
			if (!(*(arguments_[i]) == *(x.arguments_[i]))) return false;
		}
		return true;
	} else {
		return false;
	}
}

VariableSet Predicate::getVariables1() const {
	VariableSet out;
	for (auto &arg: arguments_) {
		out.insert(arg->variables().begin(), arg->variables().end());
	}
	return out;
}

size_t Predicate::hash() const {
	static const auto GOLDEN_RATIO_HASH = static_cast<size_t>(0x9e3779b9);
	auto seed = static_cast<size_t>(0);

	// Combine the hashes.
	// The function (a ^ (b + GOLDEN_RATIO_HASH + (a << 6) + (a >> 2))) is known to
	// give a good distribution of hash values across the range of size_t.
	//
	seed ^= std::hash<std::string_view>{}(functor_->stringForm()) + GOLDEN_RATIO_HASH + (seed << 6) + (seed >> 2);
	for (auto &arg: arguments_) {
		seed ^= arg->hash() + GOLDEN_RATIO_HASH + (seed << 6) + (seed >> 2);
	}

	return seed;
}

void Predicate::write(std::ostream &os) const {
	// TODO: some predicates should be written in infix notation, e.g. '=', 'is', ...

	if (arity() == 3 &&
		arguments_[1]->termType() != TermType::VARIABLE &&
		functor_->stringForm() == "triple") {
		os << *arguments_[1] << '(' << *arguments_[0] << ',' << *arguments_[2] << ')';
	} else {
		os << *functor_;
		if (!arguments_.empty()) {
			os << '(';
			for (uint32_t i = 0; i < arguments_.size(); i++) {
				Term *t = arguments_[i].get();
				os << (*t);
				if (i + 1 < arguments_.size()) {
					os << ',' << ' ';
				}
			}
			os << ')';
		}
	}
}

FunctionPtr Predicate::toFunction(const std::shared_ptr<Predicate> &predicate) {
	return std::make_shared<Function>(predicate->functor(), predicate->arguments());
}

std::shared_ptr<Predicate> Predicate::fromFunction(const FunctionPtr &fn) {
	return std::make_shared<Predicate>(fn->functor(), fn->arguments());
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::Predicate &p) //NOLINT
	{
		p.write(os);
		return os;
	}
}
