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
: CompoundFormula(FormulaType::CONJUNCTION, formulae)
{
}

Conjunction::Conjunction(const Conjunction &other, const Substitution &sub)
: CompoundFormula(other, sub)
{
}

FormulaPtr Conjunction::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<Conjunction>(
		new Conjunction(*this, sub));
}

namespace knowrob {
    FormulaPtr operator&(const FormulaPtr& phi, const FormulaPtr &psi) {
        if(phi->isBottom()) return phi;
        else if(psi->isBottom()) return psi;
        else if(phi->isTop()) return psi;
        else if(psi->isTop()) return phi;
        else if(phi->type() == FormulaType::CONJUNCTION) {
            auto formulae = ((CompoundFormula*)phi.get())->formulae();
            if(psi->type() == FormulaType::CONJUNCTION) {
                auto &psi0 = ((CompoundFormula*)psi.get())->formulae();
                formulae.insert(formulae.end(), psi0.begin(), psi0.end());
            }
            else {
                formulae.push_back(psi);
            }
            return std::make_shared<Conjunction>(formulae);
        }
        else if(psi->type() == FormulaType::CONJUNCTION) {
            auto &psi0 = ((CompoundFormula*)psi.get())->formulae();
            std::vector<FormulaPtr> formulae = {phi};
            formulae.insert(formulae.end(), psi0.begin(), psi0.end());
            return std::make_shared<Conjunction>(formulae);
        }
        else {
            return std::make_shared<Conjunction>(std::vector<FormulaPtr>({phi,psi}));
        }
    }
}
