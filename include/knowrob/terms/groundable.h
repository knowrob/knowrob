/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GROUNDABLE_H
#define KNOWROB_GROUNDABLE_H

#include <memory>
#include <sstream>
#include "Variable.h"

namespace knowrob {
	/**
	 * An interface for groundable terms that have either a particular term type
	 * or are a variable.
	 * @tparam TermType the term type.
	 */
	template<class TermType> class groundable {
	public:
		groundable() = default;

		/**
		 * @param grounded a grounding.
		 */
		groundable(const std::shared_ptr<TermType> &grounded) : grounded_(grounded) {}

		/**
		 * @param variable a variable.
		 */
		groundable(const std::shared_ptr<Variable> &variable) : variable_(variable) {}

		/**
		 * @return either the grounding or the variable.
		 */
		TermPtr operator()() const { return has_grounding() ? grounded() : variable(); }

		/**
		 * Toggle the state of this groundable to "grounded" and assign the grounding.
		 * @param grounded a grounding.
		 */
		void set_grounded(const std::shared_ptr<TermType> &grounded) { grounded_ = grounded; }

		/**
		 * Toggle the state of this groundable to "variable" and assign the variable.
		 * @param variable a variable.
		 */
		void set_variable(const std::shared_ptr<Variable> &variable) { variable_ = variable; }

		/**
		 * @return true if this groundable has a grounding.
		 */
		bool has_grounding() const { return grounded_; }

		/**
		 * @return true if this groundable has a variable.
		 */
		bool has_variable() const { return !has_grounding(); }

		/**
		 * @return the grounding or nullptr if this groundable is a variable.
		 */
		auto grounded() const { return grounded_; }

		/**
		 * @return the variable or nullptr if this groundable is grounded.
		 */
		auto variable() const { return makeVariable(); }

	protected:
		std::shared_ptr<TermType> grounded_;
		std::shared_ptr<Variable> variable_;

		std::shared_ptr<Variable> makeVariable() {
			static uint32_t varCounter = 0;
			if (!variable_) {
				variable_ = std::make_shared<Variable>((std::ostringstream() << "v_kb" << varCounter++).str());
			}
			return variable_;
		}
	};

} // knowrob

#endif //KNOWROB_GROUNDABLE_H
