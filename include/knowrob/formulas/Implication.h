/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_IMPLICATION_H
#define KNOWROB_IMPLICATION_H

#include "CompoundFormula.h"

namespace knowrob {

	class Implication : public CompoundFormula {
	public:
		explicit Implication(const FormulaPtr &antecedent, const FormulaPtr &consequent);

		const FormulaPtr &antecedent() const { return formulae_[0]; }

		const FormulaPtr &consequent() const { return formulae_[1]; }

		// Override ConnectiveFormula
		const char *operator_symbol() const override { return "->"; }

	protected:
		bool isEqual(const Formula &other) const override;
	};

} // knowrob

#endif //KNOWROB_IMPLICATION_H
