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
#include <knowrob/terms.h>
#include <knowrob/formulas/ConnectiveFormula.h>

namespace knowrob {
	/**
	 * A disjunctive expression.
	 */
	class DisjunctionFormula : public ConnectiveFormula {
	public:
		/**
		 * @formulae list of sub-formulas.
		 */
		explicit DisjunctionFormula(const std::vector<FormulaPtr> &formulae);
		
		// Override Formula
		FormulaPtr applySubstitution(const Substitution &sub) const override;
		
		// Override ConnectiveFormula
		const char* operator_symbol() const override { return "\u2228"; }
	
	protected:
		DisjunctionFormula(const DisjunctionFormula &other, const Substitution &sub);
	};
}

#endif //KNOWROB_DISJUNCTION_FORMULA_H_
