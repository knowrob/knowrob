/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_UNIFIER_H_
#define KNOWROB_UNIFIER_H_

#include "Bindings.h"

namespace knowrob {
	/**
	 * A substitution that unifies some terms.
	 */
	class Unifier : public Bindings {
	public:
		/**
		 * Compute a unifier of two terms.
		 * @t0 a term.
		 * @t1 a term.
		 */
		Unifier(const TermPtr &t0, const TermPtr &t1);
		
		/**
		 * @return true is a unifier exists.
		 */
		bool exists() const { return exists_; }
		
		/**
		 * Applies the unifier to one of the unified terms.
		 * @return an instance of the unified terms.
		 */
		TermPtr apply();
	
	protected:
		TermPtr t0_;
		TermPtr t1_;
		bool exists_;
		
		bool unify(const TermPtr &t0, const TermPtr &t1);
		bool unify(const std::shared_ptr<Variable> &var, const TermPtr &t);
	};
}

#endif //KNOWROB_UNIFIER_H_
