/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PREDICATE_H_
#define KNOWROB_PREDICATE_H_

#include <vector>
#include <memory>
#include <string>
#include "Formula.h"
#include "PredicateIndicator.h"
#include "knowrob/terms/Term.h"
#include "knowrob/terms/Substitution.h"

namespace knowrob {
	/**
	 * A predicate with a functor and a number of term arguments.
	 */
	class Predicate : public Term, public Formula {
	public:
		/**
		 * @functor the functor name.
		 * @arguments vector of predicate arguments.
		 */
		explicit Predicate(
			const std::string &functor,
			const std::vector<TermPtr> &arguments={});
		
		/**
		 * @indicator a predicate indicator reference.
		 * @arguments vector of predicate arguments.
		 */
		Predicate(
			const std::shared_ptr<PredicateIndicator> &indicator,
			const std::vector<TermPtr> &arguments);
		
		/**
		 * Substitution constructor.
		 *
		 * @other a predicate.
		 * @sub a mapping from terms to variables.
		 */
		Predicate(const Predicate &other, const Substitution &sub);

		/**
		 * Get the indicator of this predicate.
		 * @return the indicator of this predicate.
		 */
		const std::shared_ptr<PredicateIndicator>& indicator() const { return indicator_; }

		/**
		 * Get the arguments of this predicate.
		 * @return a vector of predicate arguments.
		 */
		const std::vector<TermPtr>& arguments() const { return arguments_; }

        // Override Formula
        FormulaPtr applySubstitution(const Substitution &sub) const override;
		
		// Override Term, Formula
		bool isGround() const override { return variables_.empty(); }
		
		// Override Term
		bool isAtomic() const override { return false; }

		// Override Term
		const VariableSet& getVariables() override { return variables_; }
		
		// Override Term, Formula
		void write(std::ostream& os) const override;

		// Override Term
        size_t computeHash() const override;
	
	protected:
		const std::shared_ptr<PredicateIndicator> indicator_;
		const std::vector<TermPtr> arguments_;
		const VariableSet variables_;

		VariableSet getVariables1() const;
		// Override Term
		bool isEqual(const Term &other) const override;
		bool isEqual(const Formula &other) const override;
		
		static std::vector<TermPtr> applySubstitution(
			const std::vector<TermPtr> &in,
			const Substitution &sub) ;
	};

    using PredicatePtr = std::shared_ptr<Predicate>;
}

namespace std {
    std::ostream& operator<<(std::ostream& os, const knowrob::Predicate& p);
}

#endif //KNOWROB_PREDICATE_H_
