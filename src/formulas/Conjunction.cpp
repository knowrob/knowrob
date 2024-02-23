/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <iostream>
#include <knowrob/formulas/Conjunction.h>

using namespace knowrob;

Conjunction::Conjunction(const std::vector<FormulaPtr> &formulae)
		: CompoundFormula(FormulaType::CONJUNCTION, formulae) {
}

bool Conjunction::isEqual(const Formula &other) const {
	const auto &x = static_cast<const Conjunction &>(other); // NOLINT
	if (formulae_.size() != x.formulae_.size()) return false;
	for (auto &y1: formulae_) {
		bool hasEqual = false;
		for (auto &y2: x.formulae_) {
			if (*y1 == *y2) {
				hasEqual = true;
				break;
			}
		}
		if (!hasEqual) return false;
	}
	return true;
}

namespace knowrob {
	FormulaPtr operator&(const FormulaPtr &phi, const FormulaPtr &psi) {
		if (phi->isBottom()) return phi;
		else if (psi->isBottom()) return psi;
		else if (phi->isTop()) return psi;
		else if (psi->isTop()) return phi;
		else if (phi->type() == FormulaType::CONJUNCTION) {
			auto formulae = ((CompoundFormula *) phi.get())->formulae();
			if (psi->type() == FormulaType::CONJUNCTION) {
				auto &psi0 = ((CompoundFormula *) psi.get())->formulae();
				formulae.insert(formulae.end(), psi0.begin(), psi0.end());
			} else {
				formulae.push_back(psi);
			}
			return std::make_shared<Conjunction>(formulae);
		} else if (psi->type() == FormulaType::CONJUNCTION) {
			auto &psi0 = ((CompoundFormula *) psi.get())->formulae();
			std::vector<FormulaPtr> formulae = {phi};
			formulae.insert(formulae.end(), psi0.begin(), psi0.end());
			return std::make_shared<Conjunction>(formulae);
		} else {
			return std::make_shared<Conjunction>(std::vector<FormulaPtr>({phi, psi}));
		}
	}
}
