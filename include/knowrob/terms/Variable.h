/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_VARIABLE_H_
#define KNOWROB_VARIABLE_H_

#include <string>
#include <ostream>
#include "Term.h"
#include "Atom.h"

namespace knowrob {
	/**
	 * A variable term.
	 * A variable is identified by a name string in the scope of a formula,
	 * i.e. within a formula two variables with the same name are considered to be equal.
	 */
	class Variable : public Term {
	public:
		/**
		 * @name the name of the variable.
		 */
		explicit Variable(std::string_view name);

		/**
		 * @param other another variable.
		 * @return true if this name is alphabetically before other
		 */
		bool operator<(const Variable &other) const;

		/**
		 * @param other another variable.
		 * @return true if the names of the two variables are the same.
		 */
		bool isSameVariable(const Variable &other) const;

		/**
		 * @return the name of this variable.
		 */
		std::string_view name() const { return nameAtom_->stringForm(); }

		/**
		 * @return the name of this variable.
		 */
		const AtomPtr &nameAtom() const { return nameAtom_; }

		// Override Term
		TermType termType() const final { return TermType::VARIABLE; }

		// Override Term
		bool isAtomic() const override { return false; }

		// Override Term
		const std::set<std::string_view> &variables() const override { return variables_; }

		// Override Term
		size_t hash() const override { return nameAtom_->hash(); }

	protected:
		const AtomPtr nameAtom_;
		const std::set<std::string_view> variables_;

		// Override Term
		void write(std::ostream &os) const override;
	};

	using VariablePtr = std::shared_ptr<Variable>;

	struct VariablePtrComparator {
		bool operator()(const VariablePtr &lhs, const VariablePtr &rhs) const { return *lhs < *rhs; }
	};
}

#endif //KNOWROB_VARIABLE_H_
