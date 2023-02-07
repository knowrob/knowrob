/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TYPED_REASONER_FACTORY_H_
#define KNOWROB_TYPED_REASONER_FACTORY_H_

#include <string>
#include <memory>
#include "knowrob/reasoner/IReasoner.h"
#include "knowrob/reasoner/ReasonerFactory.h"

namespace knowrob {
	/**
	 * A reasoner factory implementation for builtin reasoner types.
	 * @tparam T the type of reasoner.
	 */
	template<class T> class TypedReasonerFactory : public ReasonerFactory {
	public:
		/**
		 * @param name name of the reasoner type for which the factory can create instances.
		 */
		explicit TypedReasonerFactory(std::string name) : name_(std::move(name)) {}

		// Override ReasonerFactory
		std::shared_ptr<IReasoner> createReasoner(const std::string &reasonerID) override
		{ return std::make_shared<T>(reasonerID); }

		// Override ReasonerFactory
		const std::string& name() const override {  return name_; }
	protected:
		std::string name_;
	};
}

#endif //KNOWROB_TYPED_REASONER_FACTORY_H_
