/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_COMPOUND_FORMULA_H_
#define KNOWROB_COMPOUND_FORMULA_H_

#include <ostream>
#include <vector>
#include "knowrob/terms/Term.h"
#include <knowrob/formulas/Formula.h>

namespace knowrob {
	/**
	 * A formula with sub-formulas linked via logical connectives.
	 */
	class CompoundFormula : public Formula {
	public:
		/**
		 * @type the type of the formula.
		 * @formulae list of connected formulae.
		 */
		CompoundFormula(FormulaType type, const std::vector<FormulaPtr> &formulae);
		
		/**
		 * @return the sub-formulas associated to this formula.
		 */
		const std::vector<FormulaPtr>& formulae() const { return formulae_; }
		
		/**
		 * @return symbol string of the operator
		 */
		virtual const char* operator_symbol() const = 0;
		
		// Override Formula
		bool isGround() const override;
		
		// Override Formula
		void write(std::ostream& os) const override;
	
	protected:
		const std::vector<FormulaPtr> formulae_;
		const bool isGround_;
		
		CompoundFormula(const CompoundFormula &other, const Substitution &sub);
		
		bool isGround1() const;
		
		static std::vector<FormulaPtr> applySubstitution1(
			const std::vector<FormulaPtr> &otherFormulas,
			const Substitution &sub);
	};
}

#endif //KNOWROB_COMPOUND_FORMULA_H_
