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
#include "Term.h"
#include "Substitution.h"

namespace knowrob {
	/**
	 * The type of a predicate.
	 */
	enum class PredicateType {
		BUILT_IN = 0,
		FORMULA,
		RELATION
	};

	/**
	 * Read predicate type from term.
	 * @param term a term.
	 * @return the predicate type encoded by term.
	 */
	PredicateType predicateTypeFromTerm(const TermPtr &term);

	/**
	 * The indicator of a predicate defined by its functor and arity.
	 */
	class PredicateIndicator {
	public:
		/**
		 * @functor the functor name.
		 * @arity thr arity of this predicate.
		 */
		PredicateIndicator(std::string functor, unsigned int arity);

        // Override '==' operator
        bool operator==(const PredicateIndicator& other) const;
		// Override '<' operator
		bool operator< (const PredicateIndicator& other) const;
		
		/**
		 * Get the functor of this predicate.
		 *
		 * @return the functor name.
		 */
		const std::string& functor() const { return functor_; }
		
		/**
		 * Get the arity of this predicate.
		 *
		 * @return arity of predicate
		 */
		unsigned int arity() const { return arity_; }

		/**
		 * Convert the predicate indicator to a term of the form `'/'(Functor,Arity)`.
		 * @return the indicator as a term.
		 */
		std::shared_ptr<Term> toTerm() const;

		void write(std::ostream& os) const;
	
	private:
		const std::string functor_;
		const unsigned int arity_;
	};

	/**
	 * The description of a defined predicate.
	 */
	class PredicateDescription {
	public:
		/**
		 * @param indicator the indicator of the predicate.
		 * @param type the type of the predicate.
		 */
		PredicateDescription(const std::shared_ptr<PredicateIndicator> &indicator, PredicateType type)
		: indicator_(indicator), type_(type) {}

		/**
		 * @return the indicator of the predicate.
		 */
		const std::shared_ptr<PredicateIndicator>& indicator() const { return indicator_; }

		/**
		 * @return the type of the predicate.
		 */
		PredicateType type() const { return type_; }

	protected:
		std::shared_ptr<PredicateIndicator> indicator_;
		PredicateType type_;
	};

	/**
	 * A predicate with a functor and a number of term arguments.
	 */
	class Predicate : public Term {
	public:
		/**
		 * @functor the functor name.
		 * @arguments vector of predicate arguments.
		 */
		Predicate(
			const std::string &functor,
			const std::vector<TermPtr> &arguments);
		
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
		
		/**
		 * Create a copy of this predicate where variables are replaced by terms.
		 * @sub a mapping from variables to terms.
		 */
		std::shared_ptr<Predicate> applySubstitution(const Substitution &sub) const;
		
		// Override Term
		bool isGround() const override { return variables_.empty(); }
		
		// Override Term
		bool isAtomic() const override { return false; }

		// Override Term
		const VariableSet& getVariables() override { return variables_; }
		
		// Override Term
		void write(std::ostream& os) const override;
	
	protected:
		const std::shared_ptr<PredicateIndicator> indicator_;
		const std::vector<TermPtr> arguments_;
		const VariableSet variables_;

		VariableSet getVariables1() const;
		// Override Term
		bool isEqual(const Term &other) const override;
		
		static std::vector<TermPtr> applySubstitution(
			const std::vector<TermPtr> &in,
			const Substitution &sub) ;
	};

    using PredicatePtr = std::shared_ptr<const Predicate>;
}

#endif //KNOWROB_PREDICATE_H_
