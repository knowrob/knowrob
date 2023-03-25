/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PREDICATE_INSTANCE_H_
#define KNOWROB_PREDICATE_INSTANCE_H_

#include <memory>
#include "knowrob/terms/Term.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/formulas/Predicate.h"

namespace knowrob {
	/**
	 * The instantiation of a predicate in a reasoner module.
	 */
	class PredicateInstance {
	public:
		/**
		 * @param reasonerModule reasoner module term
		 * @param predicate a predicate instance
		 */
		PredicateInstance(const std::shared_ptr<StringTerm> &reasonerModule,
						  const std::shared_ptr<Predicate> &predicate)
		: reasonerModule_(reasonerModule), predicate_(predicate) {}

		/**
		 * @return reasoner module term
		 */
		const std::shared_ptr<StringTerm>& reasonerModule() const { return reasonerModule_; }

		/**
		 * @return a predicate instance
		 */
		const std::shared_ptr<Predicate>& predicate() const { return predicate_; }

	protected:
		const std::shared_ptr<StringTerm> reasonerModule_;
		const std::shared_ptr<Predicate> predicate_;
	};
}

#endif //KNOWROB_PREDICATE_INSTANCE_H_
