/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_VARIABLE_H_
#define KNOWROB_VARIABLE_H_

#include <string>
#include <ostream>
#include "Term.h"

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
		explicit Variable(std::string name);

		/**
		 * @param other another variable.
		 * @return true if this name is alphabetically before other
		 */
		bool operator< (const Variable& other) const;
		
		/**
		 * @return the name of this variable.
		 */
		const std::string& name() const { return name_; }
		
		// Override Term
		bool isGround() const override { return false; }
		
		// Override Term
		bool isAtomic() const override { return false; }

		// Override Term
		const VariableSet& getVariables() override { return variables_; }
		
		// Override Term
		void write(std::ostream& os) const override;

		// Override Term
        size_t computeHash() const override { return std::hash<std::string>{}(name_); }

	protected:
		const std::string name_;
		const VariableSet variables_;

		// Override Term
		bool isEqual(const Term &other) const override;
	};
}

#endif //KNOWROB_VARIABLE_H_
