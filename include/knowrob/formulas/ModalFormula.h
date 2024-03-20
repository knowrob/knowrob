/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MODAL_FORMULA_H
#define KNOWROB_MODAL_FORMULA_H

#include "CompoundFormula.h"
#include "ModalOperator.h"
#include "knowrob/TimeInterval.h"

namespace knowrob {
	/**
	 * A higher-order formula using a modal operator.
	 */
	class ModalFormula : public CompoundFormula {
	public:
		/**
		 * Create a new modal formula.
		 * @param modalOperator the modal operator.
		 * @param formula the formula.
		 */
		ModalFormula(ModalOperatorPtr modalOperator, const FormulaPtr &formula);

		/**
		 * @return the modal operator.
		 */
		const ModalOperatorPtr &modalOperator() const { return modalOperator_; }

		/**
		 * @return the modal formula.
		 */
		const FormulaPtr &modalFormula() const { return formulae_[0]; }

		/**
		 * @return true if the modal operator is a possibility operator.
		 */
		bool isModalPossibility() const { return !isModalNecessity(); }

		/**
		 * @return true if the modal operator is a necessity operator.
		 */
		bool isModalNecessity() const;

		// Override CompoundFormula
		const char *operator_symbol() const override;

		// Override Formula
		void write(std::ostream &os) const override;

	protected:
		const ModalOperatorPtr modalOperator_;

		bool isEqual(const Formula &other) const override;
	};

	namespace modals {
		/**
		 * Apply modal operator "B" to a formula.
		 * @param phi a formula.
		 * @return a modal formula.
		 */
		std::shared_ptr<ModalFormula> B(const FormulaPtr &phi);

        /**
		 * @return the belief operator `B`
		 */
		ModalOperatorPtr B();

        /**
		 * @return the belief operator `B`
		 * @param perspective a perspective IRI.
		 */
		ModalOperatorPtr B(std::string_view perspective);

        /**
		 * @return the belief operator `B`
		 * @param perspective a perspective IRI.
		 */
		ModalOperatorPtr B(std::string_view perspective, double confidence);

        /**
		 * @return the belief operator `B`
		 */
		ModalOperatorPtr B(double confidence);

		/**
		 * Apply modal operator "K" to a formula.
		 * @param phi a formula.
		 * @return a modal formula.
		 */
		std::shared_ptr<ModalFormula> K(const FormulaPtr &phi);

		/**
		 * @return the knowledge operator `K`
		 */
		ModalOperatorPtr K();

		/**
		 * @return the knowledge operator `K`
		 * @param perspective a perspective IRI.
		 */
		ModalOperatorPtr K(std::string_view perspective);

		/**
		 * Apply modal operator "P" to a formula.
		 * @param phi a formula.
		 * @return a modal formula.
		 */
		std::shared_ptr<ModalFormula> P(const FormulaPtr &phi);

        /**
		 * @return the belief operator `B`
		 */
		ModalOperatorPtr P();

        /**
		 * @return the belief operator `B`
		 */
		ModalOperatorPtr P(const TimeInterval &timeInterval);

		/**
		 * Apply modal operator "H" to a formula.
		 * @param phi a formula.
		 * @return a modal formula.
		 */
		std::shared_ptr<ModalFormula> H(const FormulaPtr &phi);

        /**
		 * @return the belief operator `H`
		 */
		ModalOperatorPtr H();

        /**
		 * @return the belief operator `H`
		 */
		ModalOperatorPtr H(const TimeInterval &timeInterval);
	} // modals
} // knowrob


#endif // KNOWROB_MODAL_FORMULA_H
