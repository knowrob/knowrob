/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GROUNDABLE_H
#define KNOWROB_GROUNDABLE_H

#include <memory>
#include <sstream>
#include "Variable.h"
#include "knowrob/Logger.h"

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
		explicit groundable(const std::shared_ptr<TermType> &grounded) : grounded_(grounded) {}

		/**
		 * @param variable a variable.
		 */
		explicit groundable(const std::shared_ptr<Variable> &variable) : variable_(variable) {}

		/**
		 * @return either the grounding or the variable.
		 */
		TermPtr operator*() const { return get(); }

		/**
		 * @return either the grounding or the variable.
		 */
		TermPtr operator()() const { return get(); }

		/**
		 * @return either the grounding or the variable.
		 */
		Term* operator->() const { return get().get(); }

		/**
		 * Toggle the state of this groundable to "grounded" and assign the grounding.
		 * @param grounded a grounding.
		 */
		groundable& operator=(const std::shared_ptr<TermType> &grounded) { set_grounded(grounded); return *this; }

		/**
		 * Toggle the state of this groundable to "variable" and assign the variable.
		 * @param variable a variable.
		 */
		groundable& operator=(const std::shared_ptr<Variable> &variable) { set_variable(variable); return *this;  }

		/**
		 * @return true if this groundable has a grounding.
		 */
		explicit operator bool() const { return has_grounding() || has_variable(); }

		/**
		 * @return either the grounding or the variable.
		 */
		TermPtr get() const {
			if (has_grounding()) {
				return grounded();
			} else {
				return variable();
			}
		}

		/**
		 * Toggle the state of this groundable to "grounded" and assign the grounding.
		 * @param grounded a grounding.
		 */
		void set_grounded(const std::shared_ptr<TermType> &grounded) { grounded_ = grounded; variable_ = nullptr; }

		/**
		 * Toggle the state of this groundable to "variable" and assign the variable.
		 * @param variable a variable.
		 */
		void set_variable(const std::shared_ptr<Variable> &variable) { grounded_ = nullptr; variable_ = variable; }

		/**
		 * @return true if this groundable has a grounding.
		 */
		bool has_grounding() const { return grounded_ != nullptr; }

		/**
		 * @return true if this groundable has a variable.
		 */
		bool has_variable() const { return !has_grounding() && variable_ != nullptr; }

		/**
		 * @return the grounding or nullptr if this groundable is a variable.
		 */
		auto grounded() const { return grounded_; }

		/**
		 * @return the variable or nullptr if this groundable is grounded.
		 */
		auto variable() const { return variable_; }

		/**
		 * Cast a term to a groundable term.
		 * @param term a term.
		 * @return a groundable term.
		 */
		static groundable<TermType> cast(const TermPtr &term) {
			if (term->isVariable()) {
				return groundable<TermType>(std::static_pointer_cast<Variable>(term));
			}
			if (auto grounded = std::dynamic_pointer_cast<TermType>(term)) {
				return groundable<TermType>(grounded);
			} else {
				KB_WARN("Cannot cast term `{}` to type `{}`.", *term, typeid(TermType).name());
				return groundable<TermType>();
			}
		}

	protected:
		std::shared_ptr<TermType> grounded_;
		std::shared_ptr<Variable> variable_;
	};

} // knowrob

#endif //KNOWROB_GROUNDABLE_H
