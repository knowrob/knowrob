/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/Disjunction.h>

using namespace knowrob;

static inline std::vector<FormulaPtr> getFormulae(const FormulaPtr &phi, const FormulaPtr &psi)
{
    if(phi->type() == FormulaType::DISJUNCTION) {
        auto phi0 = (CompoundFormula*)phi.get();
        auto formulae = phi0->formulae();
        if(psi->type() == FormulaType::DISJUNCTION) {
            auto psi0 = (CompoundFormula*)psi.get();
            auto &psiFormulae = psi0->formulae();
            formulae.insert(formulae.end(), psiFormulae.begin(), psiFormulae.end());
        }
        else {
            formulae.push_back(psi);
        }
        return formulae;
    }
    else if(psi->type() == FormulaType::DISJUNCTION) {
        auto psi0 = (CompoundFormula*)psi.get();
        auto &psiFormulae = psi0->formulae();
        std::vector<FormulaPtr> formulae = {phi};
        formulae.insert(formulae.end(), psiFormulae.begin(), psiFormulae.end());
        return formulae;
    }
    else {
        return {phi, psi};
    }
}

Disjunction::Disjunction(const std::vector<std::shared_ptr<Formula>> &formulae)
: CompoundFormula(FormulaType::DISJUNCTION, formulae)
{
}

Disjunction::Disjunction(const Disjunction &other, const Substitution &sub)
: CompoundFormula(other, sub)
{
}

bool Disjunction::isEqual(const Formula &other) const
{
	const auto &x = static_cast<const Disjunction&>(other); // NOLINT
	if(formulae_.size() != x.formulae_.size()) return false;
	for(auto &y1 : formulae_) {
		bool hasEqual=false;
		for(auto &y2 : x.formulae_) {
			if(*y1 == *y2) {
				hasEqual = true;
				break;
			}
		}
		if(!hasEqual) return false;
	}
	return true;
}

std::shared_ptr<Formula> Disjunction::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<Disjunction>(
		new Disjunction(*this, sub));
}

namespace knowrob {
    FormulaPtr operator|(const FormulaPtr& phi, const FormulaPtr &psi) {
        if(phi->isBottom()) return psi;
        else if(psi->isBottom()) return phi;
        else if(phi->isTop()) return phi;
        else if(psi->isTop()) return psi;
        else if(phi->type() == FormulaType::DISJUNCTION) {
            auto formulae = ((CompoundFormula*)phi.get())->formulae();
            if(psi->type() == FormulaType::DISJUNCTION) {
                auto &psi0 = ((CompoundFormula*)psi.get())->formulae();
                formulae.insert(formulae.end(), psi0.begin(), psi0.end());
            }
            else {
                formulae.push_back(psi);
            }
            return std::make_shared<Disjunction>(formulae);
        }
        else if(psi->type() == FormulaType::DISJUNCTION) {
            auto &psi0 = ((CompoundFormula*)psi.get())->formulae();
            std::vector<FormulaPtr> formulae = {phi};
            formulae.insert(formulae.end(), psi0.begin(), psi0.end());
            return std::make_shared<Disjunction>(formulae);
        }
        else {
            return std::make_shared<Disjunction>(std::vector<FormulaPtr>({phi,psi}));
        }
    }
}
