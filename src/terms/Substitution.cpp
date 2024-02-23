/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/Substitution.h"
#include "knowrob/terms/Unifier.h"
#include "knowrob/knowrob.h"
#include "knowrob/formulas/CompoundFormula.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/formulas/Implication.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/terms/Function.h"

using namespace knowrob;

void Reversible::rollBack() {
	while (!empty()) {
		auto &revertFunction = front();
		revertFunction();
		pop();
	}
}

bool Substitution::operator==(const Substitution &other) const {
	if (mapping_.size() != other.mapping_.size()) {
		return false;
	}

	for (auto &x: mapping_) {
		auto &val1 = x.second;
		auto &val2 = other.get(x.first);
		if (val1 && val2) {
			if (!(*val1 == *val2)) return false;
		} else if (val1 || val2) {
			return false;
		}
	}

	return true;
}

void Substitution::operator+=(const Substitution &other) {
	for (auto &x: other.mapping_) {
		if (x.second->isGround()) {
			set(x.first, x.second);
		}
	}
}

void Substitution::set(const Variable &var, const TermPtr &term) {
	mapping_.insert(std::pair<Variable, TermPtr>(var, term));
}

bool Substitution::contains(const Variable &var) const {
	return mapping_.find(var) != mapping_.end();
}

const TermPtr &Substitution::get(const Variable &var) const {
	static const TermPtr null_term;

	auto it = mapping_.find(var);
	if (it != mapping_.end()) {
		return it->second;
	} else {
		return null_term;
	}
}

const TermPtr &Substitution::get(const std::string &varName) const {
	return get(Variable(varName));
}

size_t Substitution::computeHash() const {
	auto seed = static_cast<size_t>(0);

	for (const auto &item: mapping_) {
		// Compute the hash of the key using the in-built hash function for string.
		hashCombine(seed, item.first.hash());

		auto value_hash = static_cast<size_t>(0);
		if (item.second) {
			value_hash = item.second->hash();
		}
		hashCombine(seed, value_hash);
	}

	return seed;
}

bool Substitution::unifyWith(const Substitution &other, Reversible *reversible) {
	for (const auto &pair: other.mapping_) {
		auto it = mapping_.find(pair.first);
		if (it == mapping_.end()) {
			// new variable instantiation
			auto jt = mapping_.insert(pair).first;
			if (reversible) reversible->push(([this, jt]() { mapping_.erase(jt); }));
		} else {
			// variable has already an instantiation, need to unify
			TermPtr t0 = it->second;
			TermPtr t1 = pair.second;

			// t0 and t1 are not syntactically equal -> compute a unifier
			Unifier sigma(t0, t1);
			if (sigma.exists()) {
				// a unifier exists
				it->second = sigma.apply();
				if (reversible) reversible->push(([it, t0]() { it->second = t0; }));
			} else {
				// no unifier exists
				return false;
			}
		}
	}

	return true;
}

template<class T>
std::shared_ptr<T> applyCompoundBindings( //NOLINT
		const std::shared_ptr<T> &phi,
		const Substitution &bindings) {
	std::vector<FormulaPtr> formulae;
	bool hasNewFormula = false;
	for (const auto &f: phi->formulae()) {
		auto f1 = applyBindings(f, bindings);
		if (f1 != f) {
			hasNewFormula = true;
		}
		formulae.push_back(applyBindings(f, bindings));
	}
	if (!hasNewFormula) {
		return phi;
	} else {
		return std::make_shared<T>(formulae);
	}
}

namespace knowrob {
	FormulaPtr applyBindings(const FormulaPtr &phi, const Substitution &bindings) { //NOLINT
		if (phi->isGround()) {
			return phi;
		}
		switch (phi->type()) {
			case FormulaType::MODAL: {
				auto modal = std::static_pointer_cast<ModalFormula>(phi);
				auto inner = applyBindings(modal->modalFormula(), bindings);
				if (inner == modal->modalFormula()) {
					return modal;
				} else {
					return std::make_shared<ModalFormula>(modal->modalOperator(), inner);
				}
			}
			case FormulaType::IMPLICATION: {
				auto implication = std::static_pointer_cast<Implication>(phi);
				auto antecedent = applyBindings(implication->antecedent(), bindings);
				auto consequent = applyBindings(implication->consequent(), bindings);
				if (antecedent == implication->antecedent() && consequent == implication->consequent()) {
					return implication;
				} else {
					return std::make_shared<Implication>(antecedent, consequent);
				}
			}
			case FormulaType::NEGATION: {
				auto negation = std::static_pointer_cast<Negation>(phi);
				auto negated = applyBindings(negation->negatedFormula(), bindings);
				if (negated == negation->negatedFormula()) {
					return negation;
				} else {
					return std::make_shared<Negation>(negated);
				}
			}
			case FormulaType::CONJUNCTION:
				return applyCompoundBindings<Conjunction>(std::static_pointer_cast<Conjunction>(phi), bindings);
			case FormulaType::DISJUNCTION:
				return applyCompoundBindings<Disjunction>(std::static_pointer_cast<Disjunction>(phi), bindings);
			case FormulaType::PREDICATE: {
				auto predicate = std::static_pointer_cast<Predicate>(phi);
				auto args = predicate->arguments();
				auto newArgs = std::vector<TermPtr>(args.size());
				auto hasNewArg = false;
				for (uint32_t i = 0; i < args.size(); i++) {
					auto arg = applyBindings(args[i], bindings);
					if (arg != args[i]) {
						hasNewArg = true;
					}
					newArgs[i] = arg;
				}
				if (!hasNewArg) {
					return predicate;
				} else {
					return std::make_shared<Predicate>(predicate->functor(), newArgs);
				}
			}
		}
		return phi;
	}

	TermPtr applyBindings(const TermPtr &t, const Substitution &bindings) { //NOLINT
		if (t->isGround()) {
			return t;
		}
		switch (t->termType()) {
			case TermType::ATOMIC:
				return t;
			case TermType::VARIABLE: {
				auto var = std::static_pointer_cast<Variable>(t);
				auto term = bindings.get(*var);
				if (term) {
					return term;
				} else {
					return var;
				}
			}
			case TermType::FUNCTION: {
				auto function = std::static_pointer_cast<Function>(t);
				auto args = function->arguments();
				auto newArgs = std::vector<TermPtr>(args.size());
				auto hasNewArg = false;
				for (uint32_t i = 0; i < args.size(); i++) {
					auto arg = applyBindings(args[i], bindings);
					if (arg != args[i]) {
						hasNewArg = true;
					}
					newArgs[i] = arg;
				}
				if (!hasNewArg) {
					return function;
				} else {
					return std::make_shared<Function>(function->functor(), newArgs);
				}
			}
		}
		return t;
	}
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const Substitution &omega) //NOLINT
	{
		uint32_t i = 0;
		os << '{';
		for (const auto &pair: omega) {
			if (i++ > 0) os << ',';
			os << pair.first.name() << ':' << ' ' << (*pair.second);
		}
		return os << '}';
	}
}
