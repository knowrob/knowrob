/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_CONJUNCTION_FORMULA_H_
#define KNOWROB_CONJUNCTION_FORMULA_H_

#include <vector>
#include "knowrob/terms/Term.h"
#include <knowrob/formulas/CompoundFormula.h>

namespace knowrob {
	/**
	 * A conjunction of formulae.
	 */
	class Conjunction : public CompoundFormula {
	public:
		/**
		 * @formulae list of sub-formulas.
		 */
		explicit Conjunction(const std::vector<FormulaPtr> &formulae);

		// Override ConnectiveFormula
		const char *operator_symbol() const override { return "\u2227"; }

	protected:
		bool isEqual(const Formula &other) const override;
	};

	/**
	 * Construct conjunction of formulae.
	 * @param phi a formula
	 * @param psi a formula
	 * @return conjunction of phi and psi.
	 */
	FormulaPtr operator&(const FormulaPtr &phi, const FormulaPtr &psi);
}

#endif //KNOWROB_CONJUNCTION_FORMULA_H_
