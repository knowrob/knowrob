/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ATOMIC_H
#define KNOWROB_ATOMIC_H

#include "Term.h"
#include "XSDType.h"

namespace knowrob {
	/**
	 * The type of an atomic term.
	 */
	enum class AtomicType {
		/** an atom */
		ATOM,
		/** a numeric term */
		NUMERIC,
		/** a string */
		STRING
	};

	// forward declaration
	class FramedTriple;

	/**
	 * An atomic term is a term that is not a compound term.
	 */
	class Atomic : public Term {
	public:
		Atomic() = default;

		/**
		 * @param other another atomic
		 * @return true if both atomic terms are equal
		 */
		bool isSameAtomic(const Atomic &other) const;

		/**
		 * @return the type of the atomic term
		 */
		virtual AtomicType atomicType() const = 0;

		/**
		 * Get the lexical form of this atomic term.
		 * @return the lexical form of this atomic term.
		 */
		virtual std::string_view stringForm() const = 0;

		/**
		 * Create an atomic term from a triple value.
		 * @param triple the triple
		 * @return the atomic term
		 */
		static std::shared_ptr<Atomic> makeTripleValue(const FramedTriple &triple);

		// Override Term
		size_t hash() const override;

		// Override Term
		TermType termType() const final { return TermType::ATOMIC; }

		// Override Term
		bool isAtomic() const final { return true; }

		// Override Term
		const std::set<std::string_view> &variables() const final { return Term::noVariables_; }

	protected:
		// override Term
		void write(std::ostream &os) const override { os << stringForm(); }
	};
} // knowrob

#endif //KNOWROB_ATOMIC_H
