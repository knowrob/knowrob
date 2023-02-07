/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_CONNECTIVE_FORMULA_H_
#define KNOWROB_CONNECTIVE_FORMULA_H_

#include <ostream>
#include <vector>
#include <knowrob/terms.h>
#include <knowrob/formulas/Formula.h>

namespace knowrob {
	/**
	 * A formula with sub-formulas linked via logical connectives.
	 */
	class ConnectiveFormula : public Formula {
	public:
		/**
		 * @type the type of the formula.
		 * @formulae list of connected formulae.
		 */
		ConnectiveFormula(FormulaType type, const std::vector<FormulaPtr> &formulae);
		
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
		
		ConnectiveFormula(const ConnectiveFormula &other, const Substitution &sub);
		
		bool isGround1() const;
		
		static std::vector<FormulaPtr> applySubstitution1(
			const std::vector<FormulaPtr> &otherFormulas,
			const Substitution &sub);
	};
}

#endif //KNOWROB_CONNECTIVE_FORMULA_H_
