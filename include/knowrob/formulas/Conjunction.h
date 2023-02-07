/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_CONJUNCTION_FORMULA_H_
#define KNOWROB_CONJUNCTION_FORMULA_H_

#include <vector>
#include "knowrob/terms/Term.h"
#include <knowrob/formulas/ConnectiveFormula.h>

namespace knowrob {
	/**
	 * A conjunctive expression.
	 */
	class ConjunctionFormula : public ConnectiveFormula {
	public:
		/**
		 * @formulae list of sub-formulas.
		 */
		explicit ConjunctionFormula(const std::vector<FormulaPtr> &formulae);
		
		// Override Formula
		FormulaPtr applySubstitution(const Substitution &sub) const override;
		
		// Override ConnectiveFormula
		const char* operator_symbol() const override { return "\u2227"; }
		
	protected:
		ConjunctionFormula(const ConjunctionFormula &other, const Substitution &sub);
	};
}

#endif //KNOWROB_CONJUNCTION_FORMULA_H_
