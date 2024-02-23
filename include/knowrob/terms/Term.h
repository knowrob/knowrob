/*
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
		/** atomic term */
		ATOMIC = 0,
		/** a variable */
		VARIABLE,
		/** compound term with functor and arguments */
		FUNCTION
	};

	// TODO: revise the variable handling in Term class
	// forward declaration
	class Variable;

	/**
	 * Used to compare variable pointers by value.
	 */
	struct VariableComparator {
		/**
		 * Compares Variable pointers by value.
		 */
		bool operator()(const Variable *const &v0, const Variable *const &v1) const;
	};
	// a set of const variable pointers compared by value.
	using VariableSet = std::set<const Variable *, VariableComparator>;

	/**
	 * Terms are used as components of formulas and are recursively
	 * constructed over the set of constants, variables, and function symbols.
	 */
	class Term {
	public:
		Term() = default;

		/**
		 * @param other another term
		 * @return true if both terms are equal
		 */
		bool operator==(const Term &other) const;

		/**
		 * @param other another term
		 * @return true if both terms are not equal
		 */
		bool operator!=(const Term &other) const { return !this->operator==(other); }

		/**
		 * @return the type of this term.
		 */
		virtual TermType termType() const = 0;

		/**
		 * @return true if this term has no variables.
		 */
		bool isGround() const { return variables().empty(); }

		/**
		 * @return true if this term is bound and not compound.
		 */
		virtual bool isAtomic() const = 0;

		/**
		 * @return true if this term is an atom.
		 */
		bool isAtom() const;

		/**
		 * @return true if this term is a variable.
		 */
		bool isVariable() const;

		/**
		 * @return true if this term is a function.
		 */
		bool isFunction() const;

		/**
		 * @return true if this term is a numeric.
		 */
		bool isNumeric() const;

		/**
		 * @return true if this term is a string.
		 */
		bool isString() const;

		/**
		 * @return true if this term is an IRI.
		 */
		virtual bool isIRI() const;

		/**
		 * @return true if this term is a blank node.
		 */
		virtual bool isBlank() const;

		/**
		 * @return set of variables of this term.
		 */
		virtual const VariableSet &variables() const = 0;

		/**
		 * @return the hash of this.
		 */
		virtual size_t hash() const = 0;

	protected:
		static const VariableSet noVariables_;

		/**
		 * Write the term into an ostream.
		 */
		virtual void write(std::ostream &os) const = 0;

		friend struct TermWriter;
	};

	// alias declaration
	using TermPtr = std::shared_ptr<Term>;

	/**
	 * Writes a term into an ostream.
	 */
	struct TermWriter {
		TermWriter(const Term &term, std::ostream &os) { term.write(os); }
	};
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::Term &t);
}

#endif //KNOWROB_TERM_H_
