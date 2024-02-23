/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/CompoundFormula.h>
#include <algorithm>

using namespace knowrob;

CompoundFormula::CompoundFormula(FormulaType type,
								 const std::vector<std::shared_ptr<Formula>> &formulae)
		: Formula(type),
		  formulae_(formulae),
		  isGround_(isGround1()) {
}

bool CompoundFormula::isGround1() const {
	return std::all_of(formulae_.begin(), formulae_.end(),
					   [](const std::shared_ptr<Formula> &x) { return x->isGround(); });
}

bool CompoundFormula::isGround() const {
	return isGround_;
}

void CompoundFormula::write(std::ostream &os) const {
	if (formulae_.size() == 1) {
		os << operator_symbol() << ' ';
		os << *(formulae_[0].get());
	} else {
		os << '(';
		for (uint32_t i = 0; i < formulae_.size(); i++) {
			os << *(formulae_[i].get());
			if (i + 1 < formulae_.size()) {
				os << ' ' << operator_symbol() << ' ';
			}
		}
		os << ')';
	}
}
