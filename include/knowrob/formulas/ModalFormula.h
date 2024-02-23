/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MODAL_FORMULA_H
#define KNOWROB_MODAL_FORMULA_H

#include "CompoundFormula.h"
#include "knowrob/modalities/Modality.h"
#include "knowrob/modalities/ModalOperator.h"

namespace knowrob {
	/**
	 * A formula using a unary modal operator.
	 */
	class ModalFormula : public CompoundFormula {
	public:
		ModalFormula(const ModalOperatorPtr &modalOperator, const FormulaPtr &formula);

		const ModalOperatorPtr &modalOperator() const { return modalOperator_; }

		const FormulaPtr &modalFormula() const { return formulae_[0]; }

		bool isModalPossibility() const { return !isModalNecessity(); }

		bool isModalNecessity() const;

		// Override CompoundFormula
		const char *operator_symbol() const override;

		// Override Formula
		void write(std::ostream &os) const override;

	protected:
		const ModalOperatorPtr modalOperator_;

		bool isEqual(const Formula &other) const override;
	};

	namespace modality {
		/**
		 * Apply modal operator "B" to a formula.
		 * @param phi a formula.
		 * @return a modal formula.
		 */
		std::shared_ptr<ModalFormula> B(const FormulaPtr &phi);

		/**
		 * Apply modal operator "K" to a formula.
		 * @param phi a formula.
		 * @return a modal formula.
		 */
		std::shared_ptr<ModalFormula> K(const FormulaPtr &phi);

		/**
		 * Apply modal operator "P" to a formula.
		 * @param phi a formula.
		 * @return a modal formula.
		 */
		std::shared_ptr<ModalFormula> P(const FormulaPtr &phi);

		/**
		 * Apply modal operator "H" to a formula.
		 * @param phi a formula.
		 * @return a modal formula.
		 */
		std::shared_ptr<ModalFormula> H(const FormulaPtr &phi);
	} // modality
} // knowrob


#endif // KNOWROB_MODAL_FORMULA_H
