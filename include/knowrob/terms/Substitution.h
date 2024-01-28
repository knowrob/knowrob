/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
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

namespace knowrob {
	/**
	 * Queue  of reversible operations.
	 */
	class Reversible : public std::queue<std::function<void()>> {
	public:
		/**
		 * Reverts changes made.
		 */
		void rollBack();
	};
	
	/**
	 * A substitution is a mapping from variables to terms.
	 * For example, {x1 -> t1, ..., xn -> tn} represents a substitution of
	 * each variable xi with the corresponding term ti.
	 * Applying a substitution to a term t means to replace occurrences
	 * of each xi with ti. The resulting term is referred to as an *instance* of t.
	 */
	class Substitution {
	public:
		Substitution() = default;

		explicit Substitution(std::map<Variable,TermPtr> mapping) : mapping_(std::move(mapping)) {};

		bool operator==(const Substitution &other) const;

		/**
		 * @return true if this substitution does not map a single variable to a term.
		 */
		bool empty() const { return mapping_.empty(); }

		/**
		 * @return begin iterator of substitution.
		 */
		auto begin() const { return mapping_.begin(); }

		/**
		 * @return end iterator of substitution.
		 */
		auto end() const { return mapping_.end(); }
		
		/**
		 * @var a variable.
		 * @term a term.
		 */
		void set(const Variable &var, const TermPtr &term);
		
		/**
		 * Map a variable to a term.
		 * A null pointer reference is returned if the given variable
		 * is not included in the mapping.
		 *
		 * @var a variable.
		 * @return a term reference.
		 */
		const TermPtr& get(const Variable &var) const;

		/**
		 * Map the name of a variable to a term.
		 * A null pointer reference is returned if the given variable
		 * is not included in the mapping.
		 *
		 * @var a variable.
		 * @return a term reference.
		 */
		const TermPtr& get(const std::string &varName) const;
		
		/**
		 * Returns true if the given var is mapped to a term by this substitution.
		 * @var a variable.
		 * @return true if this substitution isMoreGeneralThan the variable.
		 */
		bool contains(const Variable &var) const;

		/**
		 * Combine with another substitution.
		 * If both substitute the same variable to some term, then
		 * the combination maps to the unification of these terms, if one exists.
		 * @other another substitution
		 * @changes the diff of the substitute operation
		 * @return true if the operation succeeded.
		 */
		bool unifyWith(const Substitution &other, Reversible *reversible= nullptr);

        /**
         * @return the hash of this.
         */
		size_t computeHash() const;
	protected:
		std::map<Variable,TermPtr> mapping_;
	};
	
	// alias declaration
	using SubstitutionPtr = std::shared_ptr<Substitution>;
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Substitution& omega);
}

#endif //KNOWROB_SUBSTITUTION_H_
