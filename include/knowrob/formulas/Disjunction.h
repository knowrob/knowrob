/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DISJUNCTION_FORMULA_H_
#define KNOWROB_DISJUNCTION_FORMULA_H_

#include <vector>
#include "knowrob/terms/Term.h"
#include <knowrob/formulas/CompoundFormula.h>

namespace knowrob {
	/**
	 * A disjunctive expression.
	 */
	class Disjunction : public CompoundFormula {
	public:
		/**
		 * @formulae list of sub-formulas.
		 */
		explicit Disjunction(const std::vector<FormulaPtr> &formulae);
		
		// Override Formula
		FormulaPtr applySubstitution(const Substitution &sub) const override;
		
		// Override ConnectiveFormula
		const char* operator_symbol() const override { return "\u2228"; }
	
	protected:
		Disjunction(const Disjunction &other, const Substitution &sub);
	};

    /**
     * Construct conjunction of formulae.
     * @param phi a formula
     * @param psi a formula
     * @return conjunction of phi and psi.
     */
    FormulaPtr operator|(const FormulaPtr& phi, const FormulaPtr &psi);
}

#endif //KNOWROB_DISJUNCTION_FORMULA_H_
