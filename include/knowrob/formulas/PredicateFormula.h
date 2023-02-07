/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PREDICATE_FORMULA_H_
#define KNOWROB_PREDICATE_FORMULA_H_

#include <memory>
#include <ostream>
#include <knowrob/terms.h>
#include <knowrob/formulas/Formula.h>

namespace knowrob {
	/**
	 * A predicate formula.
	 */
	class PredicateFormula : public Formula {
	public:
		/**
		 * @predicate a predicate reference.
		 */
		explicit PredicateFormula(const std::shared_ptr<Predicate> &predicate);
		
		/**
		 * @return the predicate associated to this formula.
		 */
		const std::shared_ptr<Predicate>& predicate() const { return predicate_; }
		
		// Override Formula
		bool isGround() const override;
		
		// Override Formula
		FormulaPtr applySubstitution(const Substitution &sub) const override;
		
		// Override Formula
		void write(std::ostream& os) const override;
		
	protected:
		const std::shared_ptr<Predicate> predicate_;
	};
}

#endif //KNOWROB_PREDICATE_FORMULA_H_
