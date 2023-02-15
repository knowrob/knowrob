/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/ConnectiveFormula.h>

using namespace knowrob;

ConnectiveFormula::ConnectiveFormula(FormulaType type,
	const std::vector<std::shared_ptr<Formula>> &formulae)
: Formula(type),
  formulae_(formulae),
  isGround_(isGround1())
{
}

ConnectiveFormula::ConnectiveFormula(const ConnectiveFormula &other, const Substitution &sub)
: Formula(other.type_),
  formulae_(applySubstitution1(other.formulae_, sub)),
  isGround_(isGround1())
{
}

bool ConnectiveFormula::isGround1() const
{
	return std::all_of(formulae_.begin(), formulae_.end(),
				[](const std::shared_ptr<Formula> &x){ return x->isGround(); });
}

bool ConnectiveFormula::isGround() const
{
	return isGround_;
}

std::vector<std::shared_ptr<Formula>> ConnectiveFormula::applySubstitution1(
	const std::vector<std::shared_ptr<Formula>> &otherFormulas,
	const Substitution &sub)
{
	std::vector<std::shared_ptr<Formula>> out(otherFormulas.size());
	
	for(uint32_t i=0; i<otherFormulas.size(); i++) {
		out[i] = (otherFormulas[i]->isGround() ?
			otherFormulas[i] :
			otherFormulas[i]->applySubstitution(sub));
	}
	
	return out;
}

void ConnectiveFormula::write(std::ostream& os) const
{
	os << '(';
	for(uint32_t i=0; i<formulae_.size(); i++) {
		os << *(formulae_[i].get());
		if(i+1 < formulae_.size()) {
			os << ' ' << operator_symbol() << ' ';
		}
	}
	os << ')';
}
