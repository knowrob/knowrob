/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SUBSTITUTION_H_
#define KNOWROB_SUBSTITUTION_H_

#include <queue>
#include <functional>
#include <map>
#include <memory>
#include <ostream>
#include "Term.h"
#include "Variable.h"
#include "knowrob/formulas/Formula.h"
#include "Atomic.h"

namespace knowrob {
	/**
	 * A substitution is a mapping from variable names to terms.
	 * For example, {x1 -> t1, ..., xn -> tn} represents a substitution of
	 * each variable xi with the corresponding term ti.
	 * Applying a substitution to a term t means to replace occurrences
	 * of each xi with ti. The resulting term is referred to as an *instance* of t.
	 */
	class Substitution {
	public:
		/**
		 * A mapping from variable names to terms.
		 */
		using Map = std::map<std::string_view, std::pair<std::shared_ptr<Variable>, TermPtr>>;

		Substitution() = default;

		/**
		 * @param mapping a mapping from variables to terms.
		 */
		explicit Substitution(const std::map<std::shared_ptr<Variable>, TermPtr> &mapping);

		/**
		 * @param other another substitution.
		 * @return true if this substitution is equal to the other substitution.
		 */
		bool operator==(const Substitution &other) const;

		/**
		 * Combine with another substitution.
		 * @param other another substitution.
		 */
		void operator+=(const Substitution &other);

		/**
		 * @return true if this substitution does not map a single variable to a term.
		 */
		bool empty() const { return mapping_.empty(); }

		/**
		 * @return number of variables mapped to terms.
		 */
		auto size() const { return mapping_.size(); }

		/**
		 * @return begin iterator of substitution.
		 */
		auto begin() const { return mapping_.begin(); }

		/**
		 * @return end iterator of substitution.
		 */
		auto end() const { return mapping_.end(); }

		/**
		 * Map a variable to a term.
		 * @var a variable.
		 * @term a term.
		 */
		void set(const std::shared_ptr<Variable> &var, const TermPtr &term);

		/**
		 * Map the name of a variable to a term.
		 * A null pointer reference is returned if the given variable
		 * is not included in the mapping.
		 *
		 * @var a variable.
		 * @return a term reference.
		 */
		const TermPtr &get(std::string_view varName) const;

		/**
		 * Map the name of a variable to a term.
		 * A null pointer reference is returned if the given variable
		 * is not included in the mapping.
		 *
		 * @var a variable.
		 * @return a term reference.
		 */
		const std::shared_ptr<Atomic> getAtomic(std::string_view varName) const;

		/**
		 * Returns true if the given var is mapped to a term by this substitution.
		 * @var a variable.
		 * @return true if this substitution isMoreGeneralThan the variable.
		 */
		bool contains(std::string_view varName) const;

		/**
		 * Combine with another substitution.
		 * If both substitute the same variable to some term, then
		 * the combination maps to the unification of these terms, if one exists.
		 * @other another substitution
		 * @return true if the operation succeeded.
		 */
		bool unifyWith(const Substitution &other);

		/**
		 * @return the hash of this.
		 */
		size_t hash() const;

		/**
		 * @return a const empty substitution.
		 */
		static std::shared_ptr<const Substitution> emptySubstitution();

	protected:
		Map mapping_;
	};

	// alias declaration
	using SubstitutionPtr = std::shared_ptr<Substitution>;
	using BindingsHandler = std::function<void(const SubstitutionPtr &)>;

	/**
	 * Apply a substitution to a term.
	 * @param term a term.
	 * @param bindings a substitution.
	 * @return the term with the substitution applied.
	 */
	TermPtr applyBindings(const TermPtr &term, const Substitution &bindings);

	/**
	 * Apply a substitution to a formula.
	 * @param phi a formula.
	 * @param bindings a substitution.
	 * @return the formula with the substitution applied.
	 */
	FormulaPtr applyBindings(const FormulaPtr &phi, const Substitution &bindings);
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::Substitution &omega);
}

#endif //KNOWROB_SUBSTITUTION_H_
