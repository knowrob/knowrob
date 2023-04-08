/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TERM_H_
#define KNOWROB_TERM_H_

#include <set>
#include <memory>
#include <ostream>

namespace knowrob {
	/**
	 * The type of a term.
	 */
	enum class TermType {
		PREDICATE = 0,
		VARIABLE,
		STRING,
		DOUBLE,
		INT32,
		LONG,
		LIST,
        MODAL_OPERATOR
	};

	// forward declaration
	class Variable;
	/**
	 * Used to compare variable pointers by value.
	 */
	struct VariableComparator {
		/**
		 * Compares Variable pointers by value.
		 */
		bool operator()(const Variable* const &v0, const Variable* const &v1) const;
	};
	// a set of const variable pointers compared by value.
	using VariableSet = std::set<const Variable*,VariableComparator>;

	/**
	 * An expression in the querying language.
	 * Terms are used as components of formulas and are recursively
	 * constructed over the set of constants, variables, function symbols,
	 * and predicate symbols.
	 * Note that all terms are immutable.
	 */
	class Term {
	public:
		/**
		 * @type the type of this term.
		 */
		explicit Term(TermType type);

		/**
		 * @param other another term
		 * @return true if both terms are equal
		 */
        bool operator==(const Term& other) const;
		
		/**
		 * @return the type of this term.
		 */
		const TermType& type() const { return type_; }
		
		/**
		 * @return true if this term has no variables.
		 */
		virtual bool isGround() const = 0;
		
		/**
		 * @return true if this term is bound and not compound.
		 */
		virtual bool isAtomic() const = 0;

		/**
		 * @return set of variables of this term.
		 */
		virtual const VariableSet& getVariables() = 0;
		
		/**
		 * Write the term into an ostream.
		 */
		virtual void write(std::ostream& os) const = 0;
	
	protected:
		static const VariableSet noVariables_;
		const TermType type_;

		virtual bool isEqual(const Term &other) const = 0;
	};
	
	// alias declaration
	using TermPtr = std::shared_ptr<Term>;
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Term& t);
}

#endif //KNOWROB_TERM_H_
