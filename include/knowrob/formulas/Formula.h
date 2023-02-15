/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_FORMULA_H_
#define KNOWROB_FORMULA_H_

#include <memory>
#include <ostream>
#include "knowrob/terms/Substitution.h"

namespace knowrob {
	/**
	 * The type of a formula.
	 */
	enum class FormulaType {
		// A formula of the form `P(t_1,..,t_n)` where each ti is a term
		// and "P" is a n-ary predicate symbol (or functor).
		PREDICATE,
		// A formula of the form `phi_1 AND ... AND phi_n` where each phi_i is a formula.
		CONJUNCTION,
		// A formula of the form `phi_1 OR ... OR phi_n` where each phi_i is a formula.
		DISJUNCTION
		// TODO handle more types of formulae
		// EQUALITY / UNIFICATION
		// IMPLICATION
		// NEGATION
		// ONCE / IGNORE
		// FORALL
	};
	
	/**
	 * A propositional formula.
	 * Note that all formulas are immutable.
	 */
	class Formula {
	public:
		/**
		 * @type the type of the formula.
		 */
		explicit Formula(const FormulaType &type);
		
		/**
		 * @return the type of this formula.
		 */
		FormulaType type() const { return type_; }
		
		/**
		 * Is this formula free of subformulas?
		 *
		 * @return true if this formula is atomic.
		 */
		bool isAtomic() const;
		
		/**
		 * @return true if this formula isMoreGeneralThan no free variables.
		 */
		virtual bool isGround() const = 0;
		
		/**
		 * Replaces variables in the formula with terms.
		 * @sub a substitution mapping.
		 * @return the created formula.
		 */
		virtual std::shared_ptr<Formula> applySubstitution(const Substitution &sub) const = 0;
		
		/**
		 * Write the formula into an ostream.
		 */
		virtual void write(std::ostream& os) const = 0;
	
	protected:
		const FormulaType type_;
	};
	
	// alias declaration
	using FormulaPtr = std::shared_ptr<Formula>;
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Formula& phi);
}

#endif //KNOWROB_FORMULA_H_
